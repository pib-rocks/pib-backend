import os
import json
import asyncio
import operator
import logging
from typing import Annotated, List, TypedDict, Dict, Any, Optional

from langchain_core.messages import SystemMessage, HumanMessage, BaseMessage, AIMessage
from langgraph.graph import StateGraph, END
from langchain_mcp_adapters.client import MultiServerMCPClient
from langchain_tryb import TrybClient, TrybModel

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GraphState(TypedDict):
    messages: Annotated[List[BaseMessage], operator.add]
    tool: Dict[str, Any]  # {"tool": str, "args": {...}} oder {}


class LangchainService:
    """
    Orchestriert:
    - TrybClient/TrybModel
    - Lokale + MCP-Tools
    - LangGraph mit decide/tools/answer
    """

    # ---------- Konstruktor ----------
    def __init__(self, token: str, body: dict):
        self.token = token
        self.body = body

        # Lokale Tools
        def add(a: int, b: int) -> str:
            return str(int(a) - int(b))
        def zufall() -> str:
            return str(99)

        self.LOCAL_TOOLS: Dict[str, Any] = {
            "add": add,
            "zufall": lambda **_: zufall(),
        }

        # MCP lazy cache
        self._MCP_TOOLS: Optional[Dict[str, Any]] = None
        self._MCP_LOAD_ERROR: Optional[Exception] = None

        # Aliase
        self.ALIASES = {"gesture": "geste"}

        # Antwort-Prompt
        self.SYS_ANSWER = (
            "Antworte kurz und auf Deutsch. Nutze die gelieferte Tool-Ausgabe. "
            "Rufe KEIN Tool mehr auf und gib KEIN JSON zurück."
        )

    # ---------- Public API ----------
    def new_llm(self) -> TrybModel:
        """Erzeugt ein TrybModel auf Basis von token/body."""
        return TrybModel(TrybClient(self.token, self.body))

    def make_graph(self, llm: TrybModel):
        """Baut den LangGraph (decide -> tools -> answer)."""
        wf = StateGraph(GraphState)

        def decide_node(state: GraphState) -> GraphState:
            query = state["messages"][-1]
            ai = llm.invoke([SystemMessage(content=self.system_decide_prompt()), query])

            tool: Dict[str, Any] = {}
            try:
                data = json.loads(ai.content)
                logger.warning("Tool-Antwort (ai.content): %r", ai.content)
                if isinstance(data, dict) and "tool" in data:
                    tool = {"tool": str(data["tool"]), "args": data.get("args", {}) or {}}
            except Exception as e:
                logger.warning("Tool-Call konnte nicht als JSON gelesen werden: %s", e)

            return {"messages": [ai], "tool": tool}

        async def tool_node(state: GraphState) -> GraphState:
            d = state["tool"]
            name = d.get("tool")
            args = d.get("args", {}) or {}
            res = await self.acall_tool(name, args)

            # robust in String umwandeln (dein bisheriger Stil)
            try:
                res_str = res if isinstance(res, str) else json.dumps(res, ensure_ascii=False)
            except Exception:
                logger.info("Tool-Resultat nicht direkt serialisierbar – fallback auf str().")
                res_str = str(res)

            return {"messages": [AIMessage(content=res_str)]}

        def answer_node(state: GraphState) -> GraphState:
            # Wenn ein Tool gewählt wurde, nimm direkt dessen Output
            if state.get("tool"):
                tool = state["tool"]
                tool_name = tool.get("tool", "unbekanntes Tool")
                args = tool.get("args", {})

                # Tool-Ausgabe extrahieren
                last_ai = next((m for m in reversed(state["messages"]) if isinstance(m, AIMessage)), None)
                tool_output = last_ai.content if last_ai else ""

                # Schöne Antwort formulieren
                arg_text = ", ".join(f"{k}={v}" for k, v in args.items()) or "keine Argumente"
                antwort = f"✅ Die Geste '{tool_name}' wurde mit {arg_text} erfolgreich ausgeführt."

                if "fehler" in tool_output.lower() or "error" in tool_output.lower():
                    antwort = f"❌ Beim Ausführen von '{tool_name}' ist ein Fehler aufgetreten: {tool_output}"

                return {"messages": [AIMessage(content=antwort)]}

            # sonst: normales LLM
            msgs: List[BaseMessage] = [SystemMessage(content=self.SYS_ANSWER)]
            humans = [m for m in state["messages"] if isinstance(m, HumanMessage)]
            if humans:
                msgs.append(humans[0])
            ai2 = llm.invoke(msgs)
            return {"messages": [AIMessage(content=getattr(ai2, "content", "") or "")]}

        def route_from_decide(state: GraphState) -> str:
            return "tools" if state.get("tool") else "answer"

        wf.add_node("decide", decide_node)
        wf.add_node("tools", tool_node)
        wf.add_node("answer", answer_node)
        wf.set_entry_point("decide")
        wf.add_conditional_edges("decide", route_from_decide, {"tools": "tools", "answer": "answer"})
        wf.add_edge("tools", "answer")
        wf.add_edge("answer", END)
        return wf.compile()

    async def stream_llm_tokens(self):
        """
        Streamt Text-Chunks über einen pro-Request Graph (mit frischem LLM).
        Nutzt nur body['data'] als Eingabe-Nachricht – der TrybClient bekommt den
        kompletten Body ohnehin im Konstruktor und reicht ihn 1:1 an Tryb weiter.
        """
        llm = self.new_llm()
        graph = self.make_graph(llm)
        yielded = False
        prompt = (self.body.get("data") or ".").strip() or "."
        async for ev in graph.astream_events(
            {"messages": [HumanMessage(content=prompt)], "tool": {}},
            version="v1",
        ):
            c = (ev.get("data") or {}).get("chunk")
            if not c:
                continue
            content = getattr(c, "content", "")
            if isinstance(content, str):
                txt = content
            elif isinstance(content, list):
                txt = "".join(
                    (blk.get("text") or blk.get("content") or "")
                    for blk in content if isinstance(blk, dict)
                )
            else:
                txt = ""
            if txt:
                yielded = True
                yield txt

        if not yielded:
            out = await graph.ainvoke({"messages": [HumanMessage(content=prompt)], "tool": {}})
            ai_msgs = [m for m in out["messages"] if isinstance(m, AIMessage)]
            yield (ai_msgs[-1].content if ai_msgs else "")

    async def run_once_with(self) -> str:
        """Einmalige Ausführung ohne Streaming."""
        llm = self.new_llm()
        graph = self.make_graph(llm)
        prompt = (self.body.get("data") or ".").strip() or "."
        out = await graph.ainvoke({"messages": [HumanMessage(content=prompt)], "tool": {}})
        ai_msgs = [m for m in out["messages"] if isinstance(m, AIMessage)]
        return ai_msgs[-1].content if ai_msgs else ""

    # ---------- Hilfs-/Service-Methoden ----------
    def system_decide_prompt(self) -> str:
        tool_list = ", ".join(self.available_tool_names()) or "—"
        return (
            "Antworte kurz und auf Deutsch. Wenn eines der Tools ("
            f"{tool_list}"
            ") die Aufgabe klar löst, antworte AUSSCHLIESSLICH mit einem JSON-Objekt: "
            '{"tool":"<name>","args":{...}} und sonst mit normalem Text.'
        )

    def available_tool_names(self) -> List[str]:
        mcp = self.get_mcp_tools()
        return list(self.LOCAL_TOOLS.keys()) + list(mcp.keys())

    def _mcp_url(self) -> str:
        return os.getenv("MCP_URL", "http://andi-desktop:9292/mcp")

    def _resolve_tool_name(self, name: str) -> str:
        return self.ALIASES.get(name, name)

    def _mcp_servers(self) -> Dict[str, Dict[str, str]]:
        """
        Liest alle MCP-Server aus ENV MCP_SERVERS (JSON).
        Beispiel:
        MCP_SERVERS='{
            "math": {"url":"http://localhost:9292/mcp","transport":"streamable_http"},
            "robot":{"url":"http://192.168.0.99:9393/mcp","transport":"streamable_http"}
        }'
        Fallback (Dev): ein lokaler Server unter 9292.
        """
        raw = os.getenv("MCP_SERVERS")
        if not raw:
            # Dev-Default, damit es out-of-the-box funktioniert
            return {"srv": {"url": "http://andi-desktop:9292/mcp", "transport": "streamable_http"}}
        try:
            servers = json.loads(raw)
            if not isinstance(servers, dict):
                logger.error("MCP_SERVERS muss ein JSON-Objekt sein, erhalten: %r", type(servers).__name__)
                return {}
            return servers
        except json.JSONDecodeError as e:
            logger.error("Ungültiges MCP_SERVERS JSON: %s", e)
            return {}

    def get_mcp_tools(self) -> Dict[str, Any]:
        if self._MCP_TOOLS is not None:
            return self._MCP_TOOLS
        try:
            servers = self._mcp_servers()
            if not servers:
                logger.warning("Keine MCP-Server konfiguriert (MCP_SERVERS leer oder ungültig).")
                self._MCP_TOOLS, _MCP_LOAD_ERROR = {}, None
                return self._MCP_TOOLS

            logger.info("Lade MCP-Tools von Servern: %s", ", ".join(servers.keys()))
            client = MultiServerMCPClient(servers)
            tools = asyncio.run(client.get_tools())
            self._MCP_TOOLS = {t.name: t for t in tools}
            self._MCP_LOAD_ERROR = None
            logger.info("Geladene MCP-Tools: %s", ", ".join(self._MCP_TOOLS.keys()) or "—")
        except Exception as e:
            self._MCP_TOOLS = {}
            self._MCP_LOAD_ERROR = e
            logger.error("MCP-Tools konnten nicht geladen werden: %s", e)
        return self._MCP_TOOLS


    async def load_mcp_tools(self):
        return

    async def acall_tool(self, name: str, args: Dict[str, Any]) -> str:
        name = self._resolve_tool_name(name)
        args = args or {}

        if name in self.LOCAL_TOOLS:
            res = self.LOCAL_TOOLS[name](**args)
            return res if isinstance(res, str) else json.dumps(res, ensure_ascii=False)

        mcp_tools = self.get_mcp_tools()
        if name in mcp_tools:
            res = await mcp_tools[name].ainvoke(args)
            try:
                return res if isinstance(res, str) else json.dumps(res, ensure_ascii=False)
            except Exception:
                return str(res)

        if self._MCP_LOAD_ERROR is not None:
            return (
                f"Unbekanntes Tool: {name}. MCP-Server nicht verfügbar oder Tools nicht geladen. "
                f"Letzter Ladefehler: {self._MCP_LOAD_ERROR.__class__.__name__}: {self._MCP_LOAD_ERROR}"
            )
        return f"Unbekanntes Tool: {name}"

if __name__ == "__main__":
    import sys
    import asyncio

    async def main():
        token = os.getenv("TRYB_API_KEY", "dummy_token")
        prompt = " ".join(sys.argv[1:]).strip() or "gib mir eine Zufallszahl"

        body = {
            "data": prompt,
            "messageHistory": [],
            "personality": {"description": "", "model": "gpt-4o"},
        }

        svc = LangchainService(token, body)

        # Einmalige Ausführung (kein Streaming)
        print("\n=== run_once_with ===")
        result = await svc.run_once_with()
        print(result)

        # Streaming-Demo
        print("\n=== streaming ===")
        async for chunk in svc.stream_llm_tokens():
            print(chunk, end="", flush=True)
        print()  # Zeilenumbruch nach dem Stream

    asyncio.run(main())
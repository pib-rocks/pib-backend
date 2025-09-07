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
import traceback

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

        self.SYS_FORMAT = (
            "Formatiere die folgende Ausgabe kurz auf Deutsch. "
            "Wenn es eine JSON-Liste ist, gib nur eine knappe, kommaseparierte Liste der wichtigsten Namen aus, "
            "maximal 10 Einträge, bei mehr: ' (+N weitere)'. "
            "Kein JSON, keine Erklärungen, nur der Text."
        )

        self.SYS_POST = (
            "Der folgende Input ist ein JSON-Objekt oder eine JSON-Liste. "
            "Gib EXAKT die Listeneinträge aus, unverändert, in der Originalreihenfolge, "
            "kommasepariert in einer Zeile. "
            "Keine Übersetzungen, keine Erklärungen, keine Ergänzungen, KEIN JSON, KEIN zusätzlicher Text."
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

            # Einfacher Prompt
            sys_prompt = (
                "Wenn eine Nachricht mit einem der folgenden Tools gelöst werden kann, "
                "gib NUR den Tool-Namen zurück. Tools: "
                f"{', '.join(self.available_tool_names()) or '—'}. "
                "Sonst gib eine normale Antwort."
            )

            ai = llm.invoke([SystemMessage(content=sys_prompt), query])
            raw = (ai.content or "").strip()
            logger.info("AI Raw Output: %r", raw)

            tool = {}
            # Ganz simple Logik: Enthält Text einen Toolnamen -> Tool ausführen
            for t in self.available_tool_names():
                if t.lower() in raw.lower():
                    tool = {"tool": t, "args": {}}
                    logger.info(f"Direkt erkanntes Tool: {t}")
                    break

            return {"messages": [ai], "tool": tool}

        async def tool_node(state: GraphState) -> GraphState:
            d = state["tool"]
            name = d.get("tool")
            args = d.get("args", {}) or {}
            res = await self.acall_tool(name, args)

            try:
                res_str = res if isinstance(res, str) else json.dumps(res, ensure_ascii=False)
            except Exception:
                logger.info("Tool-Resultat nicht direkt serialisierbar – fallback auf str().")
                res_str = str(res)

            return {"messages": [AIMessage(content=res_str)]}

        def answer_node(state: GraphState) -> GraphState:
            if state.get("tool"):
                tool = state["tool"]
                tool_name = tool.get("tool", "unbekanntes Tool")
                args = tool.get("args", {})

                last_ai = next((m for m in reversed(state["messages"]) if isinstance(m, AIMessage)), None)
                tool_output = last_ai.content if last_ai else ""

                antwort = f"✅ '{tool_name}' ausgeführt. Die Antwort ist {tool_output}."

                if "fehler" in tool_output.lower() or "error" in tool_output.lower():
                    antwort = f"❌ Beim Ausführen von '{tool_name}' ist ein Fehler aufgetreten: {tool_output}"

                return {"messages": [AIMessage(content=antwort)]}

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

    def _resolve_tool_name(self, name: str) -> str:
        return self.ALIASES.get(name, name)

    def _mcp_servers(self) -> Dict[str, Dict[str, str]]:
        raw = os.getenv("MCP_SERVERS")
        if not raw:
            return {"srv": {"url": "http://ros-programs:9696/mcp", "transport": "streamable_http"}}
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
            tb = traceback.format_exc()
            logger.error("MCP-Tools konnten nicht geladen werden: %s\n%s", e, tb)
            self._MCP_TOOLS = {}
            self._MCP_LOAD_ERROR = e
        return self._MCP_TOOLS


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




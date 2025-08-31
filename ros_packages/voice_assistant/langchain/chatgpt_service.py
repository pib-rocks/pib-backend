# chatgpt_service.py
import os
import json
import asyncio
import logging
from typing import AsyncIterator, List

from langchain_openai import ChatOpenAI
from langchain_core.tools import tool
from langchain_core.messages import HumanMessage
from langchain_mcp_adapters.client import MultiServerMCPClient

# NEU: Agent + Memory
from langchain.agents import initialize_agent, AgentType
from langchain.memory import ConversationBufferMemory
from langchain.prompts import MessagesPlaceholder

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ---------------- Streaming-Helper ----------------
async def _stream_from_text(text: str) -> AsyncIterator[str]:
    """Teilt Text in kleine Chunks (simuliertes Streaming)."""
    step = 20
    for i in range(0, len(text), step):
        yield text[i:i+step]
        await asyncio.sleep(0)

# ---------------- Lokale Tools ----------------
@tool
def add(a: int, b: int) -> str:
    """Addiert zwei ganze Zahlen."""
    return str(a + b)

@tool
def random_number(min_val: int = 0, max_val: int = 100) -> str:
    """Gibt eine Zufallszahl im Bereich [min_val, max_val] zurück."""
    import random
    mn = int(min_val); mx = int(max_val)
    if mn > mx:
        mn, mx = mx, mn
    return str(random.randint(mn, mx))

# ---------------- Service ----------------
class ChatGPTService:
    """Service: OpenAI-Chat + Tools (lokal + MCP) als Agent (OpenAI Functions)."""

    def __init__(self, token: str, body: dict):
        self.token = token
        self.body = body or {}
        model = (self.body.get("personality") or {}).get("model") or os.getenv("OPENAI_MODEL", "gpt-4o-mini")
        temperature = float(os.getenv("OPENAI_TEMPERATURE", "0.2"))
        self.llm = ChatOpenAI(model=model, temperature=temperature)  # nutzt OPENAI_API_KEY
        self.local_tools = [add, random_number]
        self.mcp_tools: List = []
        self._agent = None  # wird lazy gebaut

    async def load_mcp_tools(self):
        """Lädt MCP-Tools (robust, mit Stacktrace bei Fehlern)."""
        servers_raw = os.getenv("MCP_SERVERS")
        servers = (
            json.loads(servers_raw)
            if servers_raw else
            {"default": {"url": "http://andi-desktop:9292/mcp", "transport": "streamable_http"}}
        )
        try:
            client = MultiServerMCPClient(servers)
            tools = await client.get_tools()
            self.mcp_tools = tools
            logger.info("Geladene MCP-Tools: %s", [t.name for t in tools])
        except Exception:
            logger.exception("MCP-Tools konnten nicht geladen werden")
            self.mcp_tools = []

    async def _ensure_agent(self):
        """Initialisiert den Agenten einmalig."""
        if self._agent is not None:
            return
        tools = self.local_tools + self.mcp_tools
        memory = ConversationBufferMemory(memory_key="memory", return_messages=True)
        agent_kwargs = {"extra_prompt_messages": [MessagesPlaceholder(variable_name="memory")]}
        self._agent = initialize_agent(
            tools,
            self.llm,
            agent=AgentType.OPENAI_FUNCTIONS,  # orchestriert Tool-Calls und finalisiert Text
            verbose=False,
            agent_kwargs=agent_kwargs,
            memory=memory,
        )

    async def run_once_with(self) -> str:
        """
        Führt genau einen Prompt über den Agenten aus:
        - Agent plant Tool-Calls, führt sie aus und liefert eine finale Textantwort.
        """
        prompt = (self.body.get("data") or "").strip() or "Hallo"
        await self._ensure_agent()
        try:
            result = await self._agent.ainvoke({"input": prompt})
        except Exception as e:
            logger.exception("Agent-Aufruf fehlgeschlagen")
            return f"Fehler beim Agent-Aufruf: {e}"

        if isinstance(result, dict) and "output" in result:
            return (result["output"] or "").strip() or "Keine Antwort"
        # Fallback: stringifizieren
        return (str(result) or "").strip() or "Keine Antwort"

    async def stream_llm_tokens(self) -> AsyncIterator[str]:
        """Streamt (simuliert) die Antwort in Chunks, basierend auf run_once_with()."""
        text = await self.run_once_with()
        async for chunk in _stream_from_text(text):
            yield chunk

# ---------------- Wrapper für den Server (importierbar) ----------------
async def run_once_with(body: dict, token: str) -> str:
    svc = ChatGPTService(token, body)
    await svc.load_mcp_tools()
    return await svc.run_once_with()

async def stream_llm_tokens(prompt: str) -> AsyncIterator[str]:
    token = os.getenv("TRYB_API_KEY", "dummy_token")
    body = {
        "data": prompt,
        "messageHistory": [],
        "personality": {"description": "", "model": os.getenv("OPENAI_MODEL", "gpt-4o-mini")},
    }
    svc = ChatGPTService(token, body)
    await svc.load_mcp_tools()
    async for chunk in svc.stream_llm_tokens():
        yield chunk

# ---------------- Kleiner Selbsttest ----------------
if __name__ == "__main__":
    async def main():
        prompts = [
            "Bitte addiere 5 und 7.",
            "Gib mir eine Zufallszahl.",
            "Hallo, wie geht's?",
            "wave"  # Beispiel für MCP-Tool
        ]
        for p in prompts:
            print(f"\n=== Prompt: {p} ===")
            body = {"data": p, "messageHistory": [], "personality": {"description": "", "model": "gpt-4o-mini"}}
            print("\n--- run_once_with ---")
            print(await run_once_with(body, "dummy"))
            print("\n--- stream_llm_tokens ---")
            async for c in stream_llm_tokens(p):
                print(c, end="", flush=True)
            print()

    asyncio.run(main())

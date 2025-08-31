#!/usr/bin/env python3
from __future__ import annotations
import os
import json
import asyncio
import logging
from typing import List, Optional, AsyncIterator, Dict, Any

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
# from langchain_service import LangchainService 
from chatgpt_service import ChatGPTService as LangchainService
import traceback

# ==========================================================
# Logging
# ==========================================================
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
LOG = logging.getLogger("voice")
TRACE_TOKENS = os.getenv("TRACE_TOKENS", "0") == "1"

# ==========================================================
# LangChain / LangGraph / Tryb Imports
# ==========================================================
import operator
from typing import Annotated, TypedDict
from langgraph.graph import StateGraph, END
from langchain_core.messages import SystemMessage, HumanMessage, BaseMessage, AIMessage
from langchain_mcp_adapters.client import MultiServerMCPClient

# Du hast die Tryb-Integration schon
# Falls dein Paket anders heißt, bitte hier anpassen.
from langchain_tryb import TrybClient, TrybModel


# ==========================================================
# Modelle (Request/Response)
# ==========================================================
class MessageItem(BaseModel):
    content: str
    isUser: bool = Field(...)

class Personality(BaseModel):
    description: str
    model: str 

class ChatRequest(BaseModel):
    data: str
    messageHistory: List[MessageItem] = []
    personality: Personality
    imageBase64: Optional[str] = None

# ==========================================================
# App
# ==========================================================
app = FastAPI(title="Voice Assistant Server (SSE only)", version="3.0.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ==========================================================
# Utils
# ==========================================================

def sse_event(payload: Dict[str, Any]) -> bytes:
    # korrektes SSE-Frame: "data: <json>\n\n"
    return f"data: {json.dumps(payload, ensure_ascii=False)}\n\n".encode("utf-8")

def _get_bearer(req):
    auth = (req.headers.get("authorization") or "").strip()
    return auth.split(" ", 1)[1].strip() if auth.lower().startswith("bearer ") else None


# ==========================================================
# Wrapper-Funktionen (behalten deine alten Signaturen)
# ==========================================================
async def run_once_with(body: dict, token: str) -> str:
    svc = LangchainService(token, body)
    await svc.load_mcp_tools()          # <— WICHTIG: MCP-Tools laden
    return await svc.run_once_with()

async def stream_llm_tokens(prompt: str) -> AsyncIterator[str]:
    token = os.getenv("TRYB_API_KEY", "dummy_token")
    body = {
        "data": prompt,
        "messageHistory": [],
        "personality": {"description": "", "model": "gpt-4o-mini"},  # <— echte Strings, nicht Typobjekte
    }
    svc = LangchainService(token, body)
    await svc.load_mcp_tools()          # <— WICHTIG
    async for chunk in svc.stream_llm_tokens():
        yield chunk


# ==========================================================
# Endpoint (SSE only)
# ==========================================================
async def _final_text(body: ChatRequest, bearer: Optional[str]) -> str:
    try:
        return await run_once_with(body.model_dump(), bearer or "")
    except Exception as e:
        tb = traceback.format_exc()
        LOG.error("run_once_with() failed: %s\n%s", e, tb)
        # Fallback: kurzer Mock
        return (
            f"Okay! Hier ist eine kurze Antwort auf: {body.data}. "
            "Dieser Text wird tokenweise gesendet."
        )

async def _stream_from_text(text: str):
    words = text.split(" ")
    for i, w in enumerate(words):
        yield w + (" " if i < len(words) - 1 else "")
        await asyncio.sleep(0.01)

@app.post("/voice/text")
async def voice_text(req: Request, body: ChatRequest):
    async def gen_sse():
        try:
            yield sse_event({"type": "meta", "model": body.personality.model})
            text = await _final_text(body, _get_bearer(req))   # << nur finaler Assistant-Text
            async for tok in _stream_from_text(text):          # << künstlich tokenisieren
                yield sse_event({"type": "answer", "delta": tok})
            yield sse_event({"type": "done"})
        except Exception as e:
            yield sse_event({"type": "error", "error": str(e)})
            yield sse_event({"type": "done"})

    return StreamingResponse(
        gen_sse(),
        media_type="text/event-stream; charset=utf-8",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no", "Connection": "close"},
    )

@app.get("/health")
def health():
    return {"ok": True}


# ==========================================================
# Direct run
# ==========================================================
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=9393)

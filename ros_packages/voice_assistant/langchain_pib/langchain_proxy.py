#!/usr/bin/env python3
from __future__ import annotations
import logging
from typing import List, Optional, Dict, Any

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from langchain_service import LangchainService
import traceback

# ==========================================================
# Logging
# ==========================================================
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
LOG = logging.getLogger("voice")

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
app = FastAPI(title="Voice Assistant Server (non-streaming)", version="3.0.0")
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
def _get_bearer(req: Request) -> Optional[str]:
    auth = (req.headers.get("authorization") or "").strip()
    return auth.split(" ", 1)[1].strip() if auth.lower().startswith("bearer ") else None

# ==========================================================
# Wrapper
# ==========================================================
async def run_once_with(body: dict, token: str) -> str:
    svc = LangchainService(token, body)
    return await svc.run_once_with()

async def _final_text(body: ChatRequest, bearer: Optional[str]) -> str:
    try:
        return await run_once_with(body.model_dump(), bearer or "")
    except Exception as e:
        tb = traceback.format_exc()
        LOG.error("run_once_with() failed: %s\n%s", e, tb)
        # kompakter Fallback
        return f"Okay! Hier ist eine kurze Antwort auf: {body.data}"

# ==========================================================
# Endpoint (NON-STREAMING)
# ==========================================================
@app.post("/voice/text")
async def voice_text(req: Request, body: ChatRequest):
    text = await _final_text(body, _get_bearer(req))
    # Einheitliches, simples JSON – passt zu deinem Client without SSE
    return JSONResponse(
        {"responseText": text},
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
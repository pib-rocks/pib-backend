import os, requests, json
from typing import Any, Mapping, List, Optional, Dict, Iterator
from pydantic import PrivateAttr
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import BaseMessage, AIMessage, AIMessageChunk
from langchain_core.outputs import ChatGeneration, ChatResult, ChatGenerationChunk
from langchain_core.callbacks import CallbackManagerForLLMRun

ENDPOINT = os.getenv("MY_LLM_URL", "https://platform.tryb.ai/bff/tt/be/pub/api/voice-assistant/text")

class TrybClient:
    def __init__(self, token: str, body: dict):
        self.token = token
        self.body = body
        self._session = requests.Session()

    def _build_payload(self, system_prompt: str) -> Dict[str, Any]:
        personality = (self.body.get("personality") or {}).copy()
        base_desc = (personality.get("description") or "").strip()
        sys = (system_prompt or "").strip()
        personality["description"] = base_desc if not sys else (base_desc + ("\n" if base_desc else "") + sys)
        return {**self.body, "personality": personality}

    def chat(self, messages: List[BaseMessage]) -> str:
        # System-Prompt aus Messages ziehen
        mm = [
            {"role": ("system" if m.type == "system" else "assistant" if m.type == "ai" else "user"),
             "content": m.content}
            for m in messages
        ]
        
        system_prompt = next((m["content"] for m in mm if m["role"] == "system"), "")

        headers = {
            "Accept": "text/event-stream",      # wir erwarten SSE
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.token}",
        }
        
        payload = self._build_payload(system_prompt)

        r = self._session.post(
            ENDPOINT,
            json=payload,
            headers=headers,
            timeout=60,
            stream=True,  # Streaming vom Server zulassen
        )
        r.raise_for_status()

        chunks = []
        for line in r.iter_lines():
            if not line:
                continue
            text = line.decode("utf-8", errors="ignore")
            if not text.startswith("data:"):
                continue
            chunks.append(json.loads(text[21:-1]))

        return "".join(chunks)


class TrybModel(BaseChatModel):
    model: str = "gpt-4o"
    temperature: float = 0.2
    _client: TrybClient = PrivateAttr()

    def __init__(self, client: TrybClient, **kwargs):
        super().__init__(**kwargs)
        self._client = client

    @property
    def _llm_type(self) -> str:
        return "Tryb"

    @property
    def _identifying_params(self) -> Mapping[str, Any]:
        return {"model": self.model, "temperature": self.temperature}

    def _generate(
        self,
        messages: List[BaseMessage],
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> ChatResult:
        text = self._client.chat(messages)
        return ChatResult(
            generations=[ChatGeneration(message=AIMessage(content=text))]
        )

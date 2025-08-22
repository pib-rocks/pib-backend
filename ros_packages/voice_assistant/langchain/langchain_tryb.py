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
        personality = self.body.get("personality", {}).copy()
        # immer kombinieren (append)
        personality["description"] = f"{personality.get('description', '')}\n{system_prompt}"
        payload = {**self.body, "personality": personality}
        return payload

    def _sse_text_chunks(self, r: requests.Response) -> Iterator[str]:
        """
        Liest SSE ('data: ...') und gibt jeweils einen Text-Chunk zurück.
        Erkennt '[DONE]'. Versucht JSON zu parsen und liest 'delta'/'text'/'content'.
        """
        for raw in r.iter_lines():
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line.startswith("data:"):
                continue
            data_str = line[5:].strip()
            if data_str == "[DONE]":
                break
            # robustes JSON/Plaintext-Handling
            try:
                obj = json.loads(data_str)
                text = obj.get("delta") or obj.get("text") or obj.get("content")
                if isinstance(text, str):
                    yield text
            except json.JSONDecodeError:
                # Fallback: roh weiterreichen, falls Server reinen Text sendet
                if data_str:
                    yield data_str

    def stream_response(self, messages: List[BaseMessage]) -> Iterator[str]:
        # System-Prompt extrahieren
        mm = [{"role": ("system" if m.type == "system" else "assistant" if m.type == "ai" else "user"),
               "content": m.content} for m in messages]
        system_prompt = next((m["content"] for m in mm if m["role"] == "system"), "")

        headers = {
            "Accept": "text/event-stream",
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.token}",
        }

        payload = self._build_payload(system_prompt)
        print(payload)

        r = self._session.post(ENDPOINT, json=payload, stream=True, headers=headers, timeout=60)
        r.raise_for_status()
        
        line_bin: bytes
        for line_bin in r.iter_lines():
            line_str = line_bin.decode("utf-8")
            if not line_str.startswith("data:"):
                continue
            yield json.loads(line_str[21:-1])

    def chat(self, messages: List[BaseMessage]) -> str:
        # Deltas/Chunks zu finalem String zusammensetzen
        return "".join(self.stream_response(messages))


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

    # --- Non-Streaming (invoke) ---
    def _generate(
        self,
        messages: List[BaseMessage],
        *,
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> ChatResult:
        text = self._client.chat(messages)
        if stop:
            for token in stop:
                i = text.find(token)
                if i != -1:
                    text = text[:i]
                    break
        return ChatResult(generations=[ChatGeneration(message=AIMessage(content=text))])

    # --- Streaming (model.stream(...)) ---
    def _stream(
        self,
        messages: List[BaseMessage],
        *,
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> Iterator[ChatGenerationChunk]:
        for piece in self._client.stream_response(messages):
            yield ChatGenerationChunk(message=AIMessageChunk(content=piece))

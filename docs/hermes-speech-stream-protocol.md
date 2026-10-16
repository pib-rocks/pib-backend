# Hermes / Chat action — internal speech stream protocol (PC-50)

ROS `Chat` action schema is unchanged. Only the **meaning** of `feedback.text` and
`result.text` for `TEXT_TYPE_SENTENCE` is fixed: both are **delta-only** speech
payloads for one assistant answer.

## Producer (`ChatNode._stream_chunks_to_goal`)

- Each `Chat.Feedback` with `TEXT_TYPE_SENTENCE` contains speakable text that
  was **not** present in any earlier feedback for this goal.
- `Chat.Result.text` (sentence type) contains the **remaining** speakable tail
  not yet sent in feedback. An empty string is valid and means “finalize only”.
- `TEXT_TYPE_CODE_VISUAL` feedback is unchanged (full Blockly payload per block).
- Persisted `create_chat_message` content stays **cumulative** (same message id);
  that path is independent of speech deltas.

TTFT: the first model token may still be published immediately as the first
delta when it is non-empty.

## Consumer (`StreamingSpeech.take`)

- Each call receives one producer delta (`is_final=False` for feedback,
  `is_final=True` for `Chat.Result`).
- Payloads are **not** cumulative full answers; the consumer must not merge by
  common prefix or suppress synthesis by text equality.
- Intentional repeated phrases in separate deltas are spoken again.
- `take` marks the stream finalized on the first `is_final=True` call; later
  final calls return no clauses (harmless duplicate delivery).

## Non-streamed producers

Refusal, retirement, and other single-shot results still send one full sentence
in `Chat.Result` only (one final delta). No mixed cumulative/delta paths.

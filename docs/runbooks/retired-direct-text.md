# Retired Direct text transport

New software does not call `POST /bff/tt/be/pub/api/voice-assistant/text` on
`platform.tryb.ai`. Do not enable that host in the voice egress allowlist to
bring the text route back. No request is made to it.

A personality that already stores turn-based Direct without tool calling on a model other
than pib.Cloud or the on-device model stays readable. Its channel, model,
history and credentials are not rewritten. A turn on that selection succeeds
with one German sentence and asks for a supported path: Smart, pib.Cloud, the
on-device model, or Direct with tool calling. A missing or locked provider key
does not replace that sentence. Creating or changing a personality into that
combination is rejected on the field that entered it. `toolCalling=false`
remains valid for Smart, pib.Cloud, the on-device model and native Live. Live
validation follows the resolved catalogue model, not a stale stored mode flag.

Direct tool calling, Smart/Hermes, native Live, local chat and pib.Cloud are
unchanged. Legacy speech-to-text and text-to-speech are not part of this
retirement.

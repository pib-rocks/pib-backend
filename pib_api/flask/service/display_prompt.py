"""Close the on-device password page by writing the display's hide request.

The host browser and the display node already share ``PIB_UPDATE_DIR``. This
module only writes ``display-web.json`` in the shape ``display_web_request``
validates. It does not import the display package: the flask image does not
ship that package.
"""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

REQUEST_FILENAME = "display-web.json"

PROMPT_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Operator password</title>
<style>
  body { font-family: sans-serif; margin: 2rem; background: #111; color: #f5f5f5; }
  form { display: flex; flex-direction: column; gap: 0.75rem; max-width: 28rem; }
  input { font-size: 1.25rem; padding: 0.5rem; }
  button { font-size: 1.1rem; padding: 0.6rem 1rem; }
  #notice { min-height: 1.5rem; }
</style>
</head>
<body>
<h1>Operator password</h1>
<p>Smart and Direct chats stay unavailable until this password unlocks the key store. Local voice keeps working.</p>
<form id="prompt">
  <label for="password">Password</label>
  <input id="password" name="password" type="password" autocomplete="current-password" autofocus>
  <button type="submit">OK</button>
  <button type="button" id="cancel">Cancel</button>
</form>
<p id="notice" role="status"></p>
<script>
const notice = document.getElementById("notice");
function show(text) { notice.textContent = text; }
async function post(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body)
  });
  let payload = {};
  try { payload = await response.json(); } catch (error) { payload = {}; }
  return { ok: response.ok, status: response.status, payload: payload };
}
document.getElementById("prompt").addEventListener("submit", async (event) => {
  event.preventDefault();
  const password = document.getElementById("password").value;
  if (!password) {
    show("Enter the operator password.");
    return;
  }
  const result = await post("/system/key-store/display/unlock", { password: password });
  if (result.payload.mode === "unlocked") {
    show("Unlocked.");
    return;
  }
  show(result.payload.error || "The key store is still locked. The robot stays in degraded mode.");
});
document.getElementById("cancel").addEventListener("click", async () => {
  const result = await post("/system/key-store/display/cancel", {});
  if (result.ok && result.payload.mode === "degraded") {
    show("Continuing in degraded mode.");
    return;
  }
  if (result.ok && result.payload.mode === "unlocked") {
    show("The key store stays unlocked.");
    return;
  }
  show(result.payload.error || "The password prompt could not be cancelled.");
});
</script>
</body>
</html>
"""


def dismiss_surface() -> None:
    """Ask the robot display to leave the password page.

    No ``PIB_UPDATE_DIR`` means there is no display surface to close. A failed
    write does not change the key-store result: cancel and unlock still stand.
    """
    raw = os.environ.get("PIB_UPDATE_DIR")
    if not raw:
        return
    directory = Path(raw)
    if not directory.is_dir():
        return
    document = {
        "schemaVersion": 1,
        "action": "hide",
        "url": "",
        "requestedAt": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    _atomic_write(directory, document)


def _atomic_write(directory: Path, document: dict) -> None:
    descriptor, temporary = tempfile.mkstemp(prefix=".display-web.json.", dir=directory)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(document, output, sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, directory / REQUEST_FILENAME)
        # The flask container and the host runner are different users. The
        # request is not a secret; the runner has to be able to read it.
        os.chmod(directory / REQUEST_FILENAME, 0o644)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise

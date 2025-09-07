import time
from typing import Any, Dict, Optional
from mcp.server.fastmcp import FastMCP

from .gestures import available_gestures, run_gesture
from .ros_bridge import start as ros_start, stop as ros_stop

mcp = FastMCP("pib MCP-Server", host="0.0.0.0", port=9292)

@mcp.tool(name="gesten_liste", description="Listet verfügbare Gesten.")
def gesten_liste() -> Dict[str, Any]:
    return {"gesten": available_gestures()}

@mcp.tool(
    name="geste",
    description="Führt eine Geste aus. Beispiel: geste(name='wave', args={'cycles': 3})"
)
def geste(name: str, args: Optional[Dict[str, Any]] = None):
    result = run_gesture(name, **(args or {}))
    return {"status": "ok", "gesture": name, "result": result, "args": args or {}}

def main():
    ros_start()
    try:
        mcp.run(transport="streamable-http")
    finally:
        time.sleep(0.1)
        ros_stop()

if __name__ == "__main__":
    main()
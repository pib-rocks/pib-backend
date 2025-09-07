import os
import json
import asyncio
from fastmcp import Client

URL = os.getenv("MCP_URL", "http://localhost:9292/mcp")

async def run_tool(tool: str, args_json: str | None = None, timeout: int = 10):
    payload = json.loads(args_json) if args_json else {}
    async with Client(URL) as client:
        return await asyncio.wait_for(client.call_tool(tool, payload), timeout=timeout)

async def main():
    result = await run_tool("geste", json.dumps({"name": "wave"}))
    print("is_error:", getattr(result, "is_error", None))
    print("content:", getattr(result, "content", None))
   
if __name__ == "__main__":
    asyncio.run(main())
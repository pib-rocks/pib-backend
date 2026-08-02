from setuptools import setup


setup(
    name="pib_mcp_server",
    version="0.1.0",
    description="MCP tools for pib robot capabilities",
    packages=["pib_mcp_server"],
    package_dir={"pib_mcp_server": "."},
    install_requires=["mcp", "websocket-client"],
    entry_points={"console_scripts": ["pib-mcp-server=pib_mcp_server.server:main"]},
)

# fastmcp_server_demo.py
from fastmcp import FastMCP

mcp = FastMCP("Demo 🚀")

@mcp.tool
def hello(name: str) -> str:
    return f"Hello, {name}!"

@mcp.tool
def add(a: float, b: float) -> float:
    """Add two numbers."""
    return a + b

@mcp.tool
def multiply(a: float, b: float) -> float:
    """Multiplies two numbers."""
    return a * b

if __name__ == "__main__":
    mcp.run(transport="sse", host="127.0.0.1", port=8000)  # Default: uses STDIO transport  
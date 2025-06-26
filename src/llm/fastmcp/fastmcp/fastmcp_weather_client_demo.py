
from fastmcp import Client
import asyncio

async def main():
    config = {
        "mcpServers": {
            "weather": {"url": "https://weather-api.example.com/mcp"},
            # "assistant": {"command": "python", "args": ["./assistant_server.py"]}
        }
    }

    # Create a client that connects to all servers
    client = Client(config)
    
    # 连接到SSE服务器
    async with client:
        try:
            # Access tools and resources with server prefixes
            forecast = await client.call_tool("weather_get_forecast", {"city": "ShenZheng"})
            # answer = await client.call_tool("assistant_answer_question", {"query": "What is MCP?"})
            
        except Exception as e:
            print(f"Error: {str(e)}")

# 运行异步主函数
if __name__ == "__main__":
    asyncio.run(main())
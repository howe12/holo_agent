# fastmcp_client_demo.py
from fastmcp import Client
import asyncio

async def main():
    # 连接到SSE服务器
    async with Client("http://localhost:8000/sse") as client:
        try:
            # 获取可用工具列表
            tools = await client.list_tools()
            print(f"Available tools: {[t.name for t in tools]}")
            
            # 调用add工具
            result = await client.call_tool("add", {"a": 5, "b": 3})
            print(f"Result: {result[0].text}")
            
            # 调用multiply工具
            result = await client.call_tool("multiply", {"a": 4, "b": 2.5})
            print(f"Result: {result[0].text}")
            
        except Exception as e:
            print(f"Error: {str(e)}")

# 运行异步主函数
if __name__ == "__main__":
    asyncio.run(main())
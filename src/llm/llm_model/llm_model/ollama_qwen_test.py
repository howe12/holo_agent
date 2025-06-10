import requests

url = "http://192.168.100.244:11434/api/generate"
data = {
    "model": "qwen3:4b",
    "prompt": "简述人工智能的历史",
    "stream": False  # 关闭流式响应以获取完整结果
}

response = requests.post(url, json=data)
print(response.json())  # 输出完整响应
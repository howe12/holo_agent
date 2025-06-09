import requests

url = "http://localhost:8080/completion"
headers = {
    "Content-Type": "application/json"
}
data = {
    # "prompt": "Which company released MiniCPM3?",
    # "n_predict": 128
    # "image": "/home/leo/Pictures/1.png",
    "prompt": "这张图片有什么?",
    "n_predict": 64
}

response = requests.post(url, json=data, headers=headers)

if response.status_code == 200:
    result = response.json()
    print(result["content"])
else:
    print(f"Request failed with status code {response.status_code}: {response.text}")
# Holo-Agent 

## 📌 一、项目概述

&emsp;&emsp;holo-agent​​ 是一个轻量级端侧语音交互助手，提供语音识别、语音合成、LLM理解等功能接口，旨在实现一个听你话，懂你话的机器人。

​	该项目搭载在ROS2 Humble上，依托DDS通信实现多机器人间的协同工作。

​	LLM部分，全本地部署可采用MiniPCM等GUFF格式量化模型，可在纯CPU环境中部署，如NUC迷你主机；也可将语言理解推理部分，分发至局域网中的GPU设备，Ollama本地部署LLM，搭载如DeepSeek、Qwen、Llama等LLM。



## 🧩 二、功能说明

| **模块**            | **技术方案**                                                 | **功能描述**                       |
| ------------------- | ------------------------------------------------------------ | ---------------------------------- |
| **音频录制（AUD）** | PyAudio双缓冲检测&vad识别模型                                | 检测语音开始与结束端点             |
| **语音识别（SST）** | Paraformer 流式模型（[FUNASR](https://github.com/modelscope/FunASR#installation)） | 实时转录音频为文本，支持中英文混合 |
| **语言理解（LLM）** | Ollama（[MiniPCM](https://github.com/OpenBMB/MiniCPM)）      | 解析用户指令                       |
| **语音合成（TTS）** | [Sambert-Hifigan](https://www.modelscope.cn/models/iic/speech_sambert-hifigan_tts_zh-cn_16k/file/view/master/README.md?status=1) | 低延迟语音生成，支持多语种         |



## 📋三、ToDo List

#### ⚡  **核心功能开发**

- [ ] **视觉VLM识别**：搭载VLM模型（Qwen3），进行图像识别
- [ ] **MCP部署**：统一诸多LLM的调用与回复接口格式，简化后续处理
- [ ] **DIFY RAG**: 接入RAG，更好地理解特定背景下的用户意图
- [ ] **Task 生成**：根据用户意图结合RAG，生成机器人运动的任务列表
- [ ] **ROS2 行为树**：基于任务列表执行



&emsp;&emsp;


## 🔧四、安装说明

- **语音识别模块：**[FUNASR本地语音识别](https://fvcs2dhq8qs.feishu.cn/wiki/FW0pw4ACziwjX9kUS9dc5r3jnKh?from=from_copylink&emsp;) 
- **语音合成模块：**[Sambert-Hifigan语音合成](https://fvcs2dhq8qs.feishu.cn/wiki/CwxmwHYpTilnAtkZtemcToeInyf?from=from_copylink)
- **LLM模块：**[【MiniPCM】01 安装与使用](https://fvcs2dhq8qs.feishu.cn/wiki/FOsrwjrywis8MGkElSLchl8unbc?from=from_copylink) 



## 🚀五、功能说明

### 1.语音识别模块

单独启动语音识别节点

```Python
ros2 run audio_input_local audio_input_local
```

调用语音识别功能

```Python
ros2 topic pub /llm_state std_msgs/msg/String "data: listening" --once
```

发布话题后，订阅成功后会发出“叮”的声音，开始录音，识别成功后，会发布`audio_input_audio_to_text`话题



### 2.LLM推理部分

#### **2.1 服务端启动**

- 本地MiniPCM

```Python
cd llama.cpp/
./llama-server -m ./models/minicpm3-4b-q4_k_m.gguf -c 2048 
```

- 局域网启动ollama

```Python
ollama server
```



#### 2.2 客户端请求

运行llm推理

```Python
ros2 run llm_model chatllm
```

- 本地调用MiniPCM服务

```Python
        # 调用本地MiniPCM3模型

        url = "http://localhost:8080/completion"
        headers = {
            "Content-Type": "application/json"
        }
        data = {
            # "prompt": "Which company released MiniCPM3?",
            # "n_predict": 128
            # "image": "/home/leo/Pictures/1.png",
            "prompt": str(messages_input),
            "n_predict": 128
        }
        response = requests.post(url, json=data, headers=headers)
        result = response.json()["content"]
        # self.get_logger().info(f"从MiniPCM3得到回复: {response.json()}")
        self.get_logger().info(f"\033[32m 从MiniPCM3得到回复: \n{result}\033[0m")
        
```

- ollama局域网调用qwen3:latest

```Python
import requests

        # 调用局域网ollama服务 qwen3:4b模型

        url = "http://192.168.100.244:11434/api/generate"
        data = {
            "model": "qwen3:4b",
            "prompt": str(messages_input),
            "think": False,  # 关键参数：关闭深度思考
            "stream": False  # 关闭流式响应以获取完整结果
        }

        response = requests.post(url, json=data)
        result = response.json()["response"]
        self.get_logger().info(f"\033[32m 从qwen3:4b得到回复: \n{result}\033[0m")
```

llm节点接收`llm_input_audio_to_text`话题，然后交由llm进行推理，将生成的文本回复发送

```Python
ros2 topic pub /llm_input_audio_to_text std_msgs/msg/String "data: 你是谁" --once
```



### 3.**语音合成部分**

单独运行语音合成节点

```Python
ros2 run audio_output audio_output_local.py
```

audio_output节点接收`audio_output_content`话题，然后进行语音合成播放后，将往/llm_state发布信息，启动语音识别的指令，以此循环执行交互

单独调用语音合成命令

```Python
ros2 topic pub /audio_output_content std_msgs/msg/String "data: '今天天气如何'" --once
```



## 参考

- https://github.com/Auromix/ROS-LLM




## LICENSE

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="知识共享许可协议" style="border-width:0" src="https://img.shields.io/badge/license-CC%20BY--NC--SA%204.0-lightgrey" /></a><br />本作品采用<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">知识共享署名-非商业性使用-相同方式共享 4.0 国际许可协议</a>进行许可。

## Star History

![Star History Chart](https://api.star-history.com/svg?repos=git@github.com:howe12/holo_agent.git&type=Date)

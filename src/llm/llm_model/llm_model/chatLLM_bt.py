#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2025 HuoHaiJie @NXROBO Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This code defines a ROS node called ChatLLMNode
# The node interacts with the ChatLLM service to implement conversational interactions
# The node implements the ChatLLM service callback function "llm_callback"
# The node also includes a client function "function_call_client" and a publisher "output_publisher"
# It also includes a function called "add_message_to_history" to update chat history records
# The code generates a chat response using the LLM API
# It extracts response information from the response data
# The code writes chat history records to a JSON file using Python's JSON library
# The code calls other functions using ROS Service
#
# Node test Method:
# ros2 run llm_model chatllm
# ros2 topic echo /llm_feedback_to_user
# ros2 topic pub /llm_input_audio_to_text std_msgs/msg/String "data: 'Hello,tell me a joke'" -1
#
# Author:  HuoHaiJie @NXROBO Robotics

# ROS related
import ast
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import ChatLLM,BehavioursTree
from std_msgs.msg import String

# LLM related
import json
import os
import time
from http import HTTPStatus
from random import randint
import re

# dashscope related
# import dashscope
# from dashscope import Generation, TextEmbedding, MultiModalConversation
# from dashscope.api_entities.dashscope_response import Role
from llm_config.user_config import UserConfig
import requests


# Global Initialization
config = UserConfig()
# dashscope.api_key = config.dashscope_api_key


class ChatLLMNode(Node):
    def __init__(self):
        super().__init__("ChatLLM_node")
        # 创建发布者和订阅者
        self.initialization_publisher    = self.create_publisher(String, "/llm_initialization_state", 0) # LLM初始化状态发布者
        self.llm_feedback_publisher      = self.create_publisher(String, "/llm_feedback_to_user", 0)     # LLM反馈信息发布者
        self.vlm_feedback_publisher      = self.create_publisher(String, "/vlm_feedback_to_user", 0)     # 视觉任务反馈信息发布者
        self.output_publisher            = self.create_publisher(String, "/audio_output_content", 10)    # 发布语音内容
        self.llm_state_subscriber        = self.create_subscription(String, "/llm_state", self.state_listener_callback, 0)
        self.llm_input_subscriber        = self.create_subscription(String, "/llm_input_audio_to_text", self.llm_callback, 0) # 监听语音识别结果，进行LLM推理处理

        # 创建方法调用的客户端,分为机械臂基础模式和视觉任务模式
        self.function_call_client        = self.create_client(ChatLLM, "/ChatLLM_function_call_service")
        self.vlm_function_call_client    = self.create_client(ChatLLM, "/MotionRobot_function_call_service")
        self.function_call_requst        = ChatLLM.Request()  
        self.vlm_function_call_requst    = ChatLLM.Request()
        self.get_logger().info(f"\033[36m ChatLLM Function Call Server is ready\033[0m")

        # 创建行为树的服务端
        self.srv = self.create_service(BehavioursTree, "/llm_input", self.llm_srv_callback)

        # 保存对话记录,对话的最大限制在 user_config.chat_history_max_length
        self.start_timestamp    = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.chat_history_file  = os.path.join(config.chat_history_path, f"chat_history_{self.start_timestamp}.json")
        self.write_chat_history_to_json()
        self.get_logger().info(f"\033[36m Chat history saved to {self.chat_history_file}\033[0m")

        # Function name
        self.function_name = "null"
        # Initialization ready
        self.publish_string("llm_model_processing", self.initialization_publisher)

    def state_listener_callback(self, msg):
        self.get_logger().debug(f"model node get current State:{msg}")
        # TODO

    def publish_string(self, string_to_send, publisher_to_use):
        """
        将给定的字符串消息发布到指定的ROS发布者。

        :param string_to_send: 要发布的字符串消息
        :param publisher_to_use: 用于发布消息的ROS发布者对象
        """
        # 创建一个ROS String消息对象
        msg = String()
        # 将传入的字符串赋值给消息对象的data字段
        msg.data = string_to_send

        # 使用指定的发布者发布消息
        publisher_to_use.publish(msg)
        # 记录日志，显示发布消息的话题名称和消息内容
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )

    def add_message_to_history(
        self, role, content="null", function_call=None, name=None
    ):
        """
        将一个新的消息元素对象添加到聊天历史记录中。
        该消息元素对象包含指定的角色、内容和函数调用信息。
        消息元素对象是一个字典，包含 "role"、"content" 和 "function_call" 等键值对。
        如果聊天历史记录超过了允许的最大长度，将移除最旧的消息元素对象。

        :param role: 消息的角色，例如 "user", "assistant", "tool" 等
        :param content: 消息的内容，默认为 "null"
        :param function_call: 函数调用信息，默认为 None
        :param name: 函数的名称，默认为 None
        :return: 更新后的聊天历史记录列表
        """
        # 创建包含给定选项的消息字典
        message_element_object = {
            "role": role,
            "content": content,
        }
        # 如果提供了函数名称，将其添加到消息字典中
        if name is not None:
            message_element_object["name"] = name
        # 如果提供了函数调用信息，将其添加到消息字典中
        if function_call is not None:
            message_element_object["function_call"] = function_call
        # 将消息元素对象添加到聊天历史记录中
        config.chat_history.append(message_element_object)
        # 记录日志，显示聊天历史记录已更新
        self.get_logger().info(f"Chat history updated with {message_element_object}")
        # 检查聊天历史记录是否过长
        if len(config.chat_history) > config.chat_history_max_length:
            # 记录日志，显示将移除最旧的消息
            self.get_logger().info(
                f"Chat history is too long, popping the oldest message: {config.chat_history[0]}"
            )
            # 移除最旧的消息
            config.chat_history.pop(0)

        # 返回更新后的聊天历史记录
        return config.chat_history

    def write_chat_history_to_json(self):
        """
        将聊天历史记录写入JSON文件。

        :return: 若写入成功返回True，若发生IO错误返回False
        """
        try:
            # 将聊天历史记录转换为格式化的JSON字符串，indent=4使输出的JSON文件具有更好的可读性
            json_data = json.dumps(config.chat_history, indent=4, ensure_ascii=False)

            # 以写入模式打开指定的JSON文件，使用UTF-8编码
            with open(self.chat_history_file, "w", encoding="utf-8") as file:
                # 将JSON字符串写入文件
                file.write(json_data)

            # 记录日志，提示聊天历史记录已成功写入JSON文件
            self.get_logger().info("Chat history has been written to JSON")
            return True

        except IOError as error:
            # 记录日志，提示在将聊天历史记录写入JSON文件时发生错误，并打印具体错误信息
            self.get_logger().error(f"Error writing chat history to JSON: {error}")
            return False

    def handle_dify_stream_response(self,response):
        full_text = ""  # 累积完整回复
        metadata = None  # 存储元数据
        
        if response.status_code == 200:
            for line in response.iter_lines():
                if line:
                    # 输出原始数据
                    self.get_logger().info(f"原始数据: {line}")
                    decoded_line = line.decode('utf-8').strip() # 解码并移除首尾空格
                    
                    # 仅处理有效事件行
                    if decoded_line.startswith('data:'):
                        try:
                            event_data = json.loads(decoded_line[5:])  # 移除"data:"前缀
                            event_type = event_data.get("event")
                            
                            # 处理文本分块
                            if event_type == "agent_message":
                                chunk = event_data.get("answer", "")
                                full_text += chunk
                                # 实时输出效果（可选）
                                self.get_logger().info(f"收到分块: {chunk}")
                            
                            # 捕获结束事件及元数据
                            elif event_type == "message_end":
                                metadata = event_data.get("metadata", {})
                                self.get_logger().info(f"最终元数据: {metadata}")
                        
                        except json.JSONDecodeError:
                            self.get_logger().error(f"JSON解析失败: {decoded_line}")
        else:
            self.get_logger().error(f"请求失败，状态码: {response.status_code}")
            return False

        # 返回完整结果
        return {
            "content": full_text,
            "metadata": metadata
        }

    def generate_llm_response(self, messages_input):
        """
        根据提供的输入消息生成ChatLLM的响应。
        所有参数都可以在llm_config/user_config.py文件中找到。
        """

        # 记录日志，发送消息给Qwen模型
        # self.get_logger().info(f"发送消息到Qwen: {messages_input}")

        # --------------------------------------------------------------
        # 调用在线'qwen-turbo'模型

        # response = Generation.call(
        #     model='qwen-turbo',  # 使用qwen-turbo模型
        #     messages=messages_input,  # 输入消息
        #     seed=randint(1, 10000),  # 随机种子，确保生成的结果具有一定随机性
        #     temperature=config.llm_temperature,  # 控制生成结果的多样性
        #     top_p=config.llm_top_p,  # 样本累积概率，用于多样性采样
        #     top_k=config.llm_top_k,  # 限制采样的词汇数量，用于控制输出质量
        #     result_format='message',  # 生成的结果格式为消息
        # )

        # --------------------------------------------------------------
        # 调用本地MiniPCM3模型

        # url = "http://localhost:8080/completion"
        # headers = {
        #     "Content-Type": "application/json"
        # }
        # data = {
        #     # "prompt": "Which company released MiniCPM3?",
        #     # "n_predict": 128
        #     # "image": "/home/leo/Pictures/1.png",
        #     "prompt": str(messages_input),
        #     "n_predict": 128
        # }
        # response = requests.post(url, json=data, headers=headers)
        # result = response.json()["content"]
        # # self.get_logger().info(f"从MiniPCM3得到回复: {response.json()}")
        # self.get_logger().info(f"\033[32m 从MiniPCM3得到回复: \n{result}\033[0m")
        
        # --------------------------------------------------------------
        # 调用局域网ollama服务 qwen3:4b模型

        # url = "http://192.168.100.244:11434/api/generate"
        # data = {
        #     "model": "qwen3:4b",
        #     "prompt": str(messages_input),
        #     "think": False,  # 关键参数：关闭深度思考
        #     "stream": False  # 关闭流式响应以获取完整结果
        # }

        # response = requests.post(url, json=data)
        # result = response.json()["response"]
        # self.get_logger().info(f"\033[32m 从qwen3:4b得到回复: \n{result}\033[0m")
        # --------------------------------------------------------------
        # 调用dify服务
        
        url = "http://192.168.100.244/v1/chat-messages" 
        headers = {
            # "Authorization": "Bearer app-H17Brmyq2zCaOo1UrCGijOYI",  # 产品说明小助手
            "Authorization": "Bearer app-paULCxxuHjriA97Z2zv9LZEw",  # AI MCP
            "Content-Type": "application/json"  
        }
        data = {
            "inputs": {},
            "query": str(messages_input),
            "response_mode":"streaming",
            "user":"leo",
        }

        response = requests.post(url, headers=headers,json=data,stream=True)
        
        # 针对streaming格式处理
        result = self.handle_dify_stream_response(response)
        # self.get_logger().info(f"dify回复状态码: {result['status_code']}")
        self.get_logger().info(f"\033[32m完整回复: {result['content']}\033[0m")
        self.get_logger().info(f"知识库引用: {result['metadata'].get('retriever_resources', [])}")

        if result:
            self.get_logger().info(f"\033[36m Qwen响应 : \n{result['content']}\033[0m")
            return result['content']
        else:
            return None


        # self.get_logger().info(f"\033[32m 从dify得到回复: \n{result}\033[0m")
        # --------------------------------------------------------------


        # 记录日志，检查响应的状态
        # if response.status_code == HTTPStatus.OK:

        # if response.status_code == 200:
        #     # 如果响应成功，记录响应内容
        #     self.get_logger().info(f"\033[36m Qwen响应 : \n{response}\033[0m")
        #     return response
        # else:
        #     # 如果响应失败，记录错误信息
        #     self.get_logger().error('请求ID: %s, 状态码: %s, 错误码: %s, 错误消息: %s' % (
        #         # response.request_id, response.status_code,
        #         # response.code, response.message
        #     ))
        #     return None

    def get_response_information(self, llm_response):
        """
        从ChatLLM的响应中提取响应信息。
        响应信息包括消息文本、函数调用信息和函数调用标志。
        function_flag = 0: 无函数调用，1: 有函数调用

        :param llm_response: ChatLLM服务返回的响应对象
        :return: 消息文本、函数调用信息、函数调用标志
        """

        #################################################
        ###             1. 提取响应内容，初始化标签       ###
        #################################################  
        # 从响应中获取消息内容
        # chunk = llm_response.output.choices[0]['message']['content']
        # chunk = llm_response.json()["response"] # ollama
        # chunk = llm_response.json()["answer"] # ollama
        chunk = llm_response # dify
        # 初始化消息文本，默认为None
        content = None
        # 初始化函数调用信息，默认为None
        function_call = None
        # 初始化函数调用标志，0表示无函数调用，1表示有函数调用
        function_flag = 0


        #################################################
        ###             2. 处理消息格式，提取方法名       ###
        #################################################  
        # 新增：处理包含 <think> 标签的情况
        start_tag = "<think>"
        end_tag = "</think>"
        if start_tag in chunk and end_tag in chunk:

            chunk = re.sub(r'<think>.*?</think>', '', chunk, flags=re.DOTALL).strip()
            # start_index = chunk.find(start_tag) + len(start_tag)
            # end_index = chunk.find(end_tag)
            # # 提取标签内的内容并去除首尾空格
            # content = chunk[start_index:end_index].strip()
            # return content, function_call, function_flag

        # 检查消息内容是否以 ```json 开头或包含 ```json
        if chunk.startswith('```json') or chunk.find('```json') != -1:
            # 找到 ```json 标记的结束位置
            start_index = chunk.find('```json') + len('```json')
            # 找到代码块结束标记 \n``` 的位置
            end_index = chunk.find('\n```')
            # 若起始和结束位置都有效
            if start_index != -1 and end_index != -1:
                # 提取代码块内容并去除首尾空格
                function_call = chunk[start_index:end_index].strip()
                try:
                    # 尝试将提取的内容解析为JSON对象
                    function_call = json.loads(function_call)
                    # 解析成功，设置函数调用标志为1
                    function_flag = 1
                except json.JSONDecodeError:
                    # 解析失败，不做处理
                    pass

        # 如果消息内容不以 ```json 开头，但包含 ``` 标记
        elif chunk.find('```') != -1:
            # 找到 ``` 标记的结束位置
            start_index = chunk.find('```') + 3
            # 找到下一个 ``` 标记的位置
            end_index = chunk.find('```', start_index)
            # 若起始和结束位置都有效
            if start_index != -1 and end_index != -1:
                # 提取代码块内容并去除首尾空格
                function_call = chunk[start_index:end_index].strip()
                try:
                    # 尝试将提取的内容解析为JSON对象
                    function_call = json.loads(function_call)
                    # 解析成功，设置函数调用标志为1
                    function_flag = 1
                except json.JSONDecodeError:
                    # 解析失败，不做处理
                    pass

        # 如果消息内容包含 'function_call'
        elif chunk.find('function_call') != -1:
            try:
                # 将Python字典字符串转换为字典对象
                data_dict = ast.literal_eval(chunk.strip())
                # 将字典对象转换为JSON字符串
                json_data = json.dumps(data_dict)
                # 尝试将JSON字符串解析为JSON对象
                function_call = json.loads(json_data)
                # 解析成功，设置函数调用标志为1
                function_flag = 1
            except json.JSONDecodeError:
                # 解析失败，不做处理
                pass

        # 如果消息内容中没有找到任何 ``` 标记，直接将原始文本作为消息内容
        else:
            content = chunk.strip()


        #################################################
        ###             3. 确定响应类型                 ###
        #################################################  
        # 根据content是否为None判断响应类型
        if content is not None:
            # content不为None，响应类型为文本
            function_flag = 0
            self.get_logger().info("LLM response type: TEXT")
        else:
            # content为None，响应类型为函数调用
            function_flag = 1
            self.get_logger().info("LLM response type: FUNCTION CALL")

        # 记录从LLM获取的消息文本及其类型
        self.get_logger().info(
            f"Get content from LLM: {content}, type: {type(content)}"
        )
        # 记录从LLM获取的函数调用信息及其类型
        self.get_logger().info(
            f"Get function call from LLM: {function_call}, type: {type(function_call)}"
        )

        # 示例返回值格式
        # {'function_call': [{'name': 'vlm_pick_and_place', 'params': {'object_1': '红色方块', 'object_2': '蓝色方块'}}]}
        # [{'name': 'vlm_pick_and_place', 'params': {'object_1': '红色方块', 'object_2': '蓝色方块'}}]
        return content, function_call, function_flag

    def separate_vlm_functions(self,function_calls):
        """
        检查参数中是否包含vlm,将llm与vlm涉及的方法分开。
        """
        vlm_functions = []
        llm_functions = []

        # 遍历function_calls中的每个元素
        for func in function_calls:
            # 检查'name'字段是否包含'vlm'
            if 'vlm' in func['name']:
                vlm_functions.append(func)
            else:
                llm_functions.append(func)

        return vlm_functions, llm_functions

    def function_call(self, function_call_input):
        """
        根据提供的输入参数，往机器人行为服务器发送函数调用请求，并等待响应。
        当收到响应时，将调用函数调用响应回调（callback）,返回文本响应。
        """

        # 1. 检查输入是列表类型还是字典类型，若为字典类型(包含方法)，则提取其中的 'function_call'
        if isinstance(function_call_input, list):
            fc = function_call_input
        elif isinstance(function_call_input, dict):
            fc = function_call_input['function_call']

        # 2. 将函数调用分为 VLM 和 LLM 函数调用列表
        vlm_functions, llm_functions = self.separate_vlm_functions(fc)

        # 3. 如果存在 LLM 函数调用
        if llm_functions:
            # 获取 LLM 函数的名称列表
            self.function_name = [item['name'] for item in llm_functions]
            # 将 LLM 函数调用转换为 JSON 格式，并赋值给请求文本
            self.function_call_requst.request_text = json.dumps(llm_functions)
            self.get_logger().info(f"\033[32m 请求 ChatLLM 服务端 : \n{self.function_call_requst.request_text}\033[0m")
            # 异步调用 LLM 函数请求，并设置回调函数
            future = self.function_call_client.call_async(self.function_call_requst)
            future.add_done_callback(self.function_call_response_callback)

        # 4. 如果存在 VLM 函数调用
        if vlm_functions:
            # 将 VLM 函数调用转换为 JSON 格式，并赋值给请求文本
            self.vlm_function_call_requst.request_text = json.dumps(vlm_functions)
            self.get_logger().info(f"\033[32m 请求 MotionRobot_function_call_service: \n{self.vlm_function_call_requst.request_text}\033[0m")
            # 异步调用 VLM 函数请求，并设置回调函数
            future = self.vlm_function_call_client.call_async(self.vlm_function_call_requst)
            future.add_done_callback(self.vlm_function_call_response_callback)

    def function_call_response_callback(self, future):
        """
        当收到函数调用响应时，调用此函数调用响应回调。
        该回调函数会再次调用GPT服务，以获取给用户的文本响应。

        :param future: 异步函数调用的未来对象，包含调用结果
        """
        try:
            # 从未来对象中获取函数调用的响应结果
            response = future.result()
            # 记录从ChatLLM_function_call_service收到的响应信息
            self.get_logger().info(
                f"Response from ChatLLM_function_call_service: {response}"
            )

        except Exception as e:
            # 若获取响应结果时出现异常，记录ChatLLM函数调用服务失败的信息及异常内容
            self.get_logger().info(f"ChatLLM function call service failed {e}")

        # 初始化响应文本，默认为 "null"
        response_text = "null"
        # 将响应结果转换为字符串，并通过llm_feedback_publisher发布
        self.publish_string(str(response), self.llm_feedback_publisher)

    def vlm_function_call_response_callback(self, future):
        """
        当收到视觉语言模型（VLM）相关的函数调用响应时，调用此函数调用响应回调。
        该回调函数会再次调用GPT服务，以获取给用户的文本响应。

        :param future: 异步函数调用的未来对象，包含调用结果
        """
        try:
            # 从未来对象中获取函数调用的响应结果
            response = future.result()
            # 记录异步请求成功的信息及返回结果
            self.get_logger().info(
                f"异步请求成功,返回结果: {response}"
            )

        except Exception as e:
            # 若获取响应结果时出现异常，记录异步请求失败的信息及异常内容
            self.get_logger().info(f"异步请求失败 {e}")

        # 将响应结果转换为字符串，并通过vlm_feedback_publisher发布
        self.publish_string(str(response), self.vlm_feedback_publisher)

    def llm_srv_callback(self, request, response):
        """
        llm_callback函数在ChatLLM服务被调用时触发。
        该函数是ChatLLM节点的主要功能,用于处理输入消息并生成LLM响应。
        """
        try:
            # 1. 显示收到的输入消息
            self.get_logger().info(f"\033[32m 收到的输入消息: {request.input_data}\033[0m")

            # 2. 将用户消息添加到聊天历史记录中,根据对话历史生成聊天回复
            user_prompt = "/nothink" + request.input_data
            self.add_message_to_history("user", user_prompt)

            # 3. 生成LLM响应
            llm_response = self.generate_llm_response(config.chat_history)

            if llm_response: # 检查是否生成了响应
                # 4. 提取响应的相关信息 (文本、函数调用、函数标志)
                text, function_call, function_flag = self.get_response_information(llm_response)

                # 5. 将LLM的回复添加到聊天历史记录中,写入到JSON文件
                self.add_message_to_history(role="assistant", content=text if function_flag == 0 else str(function_call))
                self.write_chat_history_to_json()
                self.get_logger().info(f"\033[36m 状态: 输出处理中\033[0m") # 显示状态: 输出处理中

                # 6. 如果存在函数调用
                if function_flag == 1:
                    # 6.1 执行机器人函数调用
                    self.function_call(function_call)
                    self.get_logger().info(f"\033[36m 状态: 函数执行中\033[0m")
                
                # 7. 如果只是文本响应
                else:
                    # 记录状态: 用户反馈
                    # self.publish_string(text, self.llm_feedback_publisher)  # 发布LLM反馈内容
                    # self.publish_string(text, self.output_publisher)        # 发布语音播放内容
                    self.get_logger().info(f"\033[36m 状态: 对话模式\033[0m")
                    response.success = True
                    response.output_data = text
                    self.get_logger().info(f"LLM processing success: {text}")
                    return response
                    
            else:
                # 如果没有生成响应，则从聊天历史中移除最后一条消息
                config.chat_history.pop()
        except Exception as e:
            self.get_logger().error(f"Service call exception: {str(e)}")
            response.success = False
            return response

    def llm_callback(self, msg):
        """
        llm_callback函数在ChatLLM服务被调用时触发。
        该函数是ChatLLM节点的主要功能,用于处理输入消息并生成LLM响应。
        """
        # 1. 显示收到的输入消息
        self.get_logger().info(f"\033[32m 收到的输入消息: {msg.data}\033[0m")

        # 2. 将用户消息添加到聊天历史记录中,根据对话历史生成聊天回复
        user_prompt = "/nothink" + msg.data
        self.add_message_to_history("user", user_prompt)

        # 3. 生成LLM响应
        llm_response = self.generate_llm_response(config.chat_history)

        if llm_response: # 检查是否生成了响应
            # 4. 提取响应的相关信息 (文本、函数调用、函数标志)
            text, function_call, function_flag = self.get_response_information(llm_response)

            # 5. 将LLM的回复添加到聊天历史记录中,写入到JSON文件
            self.add_message_to_history(role="assistant", content=text if function_flag == 0 else str(function_call))
            self.write_chat_history_to_json()
            self.get_logger().info(f"\033[36m 状态: 输出处理中\033[0m") # 显示状态: 输出处理中

            # 6. 如果存在函数调用
            if function_flag == 1:
                # 6.1 执行机器人函数调用
                self.function_call(function_call)
                self.get_logger().info(f"\033[36m 状态: 函数执行中\033[0m")
            
            # 7. 如果只是文本响应
            else:
                # 记录状态: 用户反馈
                self.publish_string(text, self.llm_feedback_publisher)  # 发布LLM反馈内容
                self.publish_string(text, self.output_publisher)        # 发布语音播放内容
                self.get_logger().info(f"\033[36m 状态: 对话模式\033[0m")
                
        else:
            # 如果没有生成响应，则从聊天历史中移除最后一条消息
            config.chat_history.pop()


def main(args=None):
    rclpy.init(args=args)
    ChatLLM = ChatLLMNode()
    rclpy.spin(ChatLLM)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

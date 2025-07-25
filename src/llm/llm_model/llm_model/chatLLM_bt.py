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
# Author:  HuoHaiJie @NXROBO Robotics

# ROS related
import ast
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import ChatLLM,BehavioursTree,ChildTree
from std_msgs.msg import String

# LLM related
import json
import os
import time
from http import HTTPStatus
from random import randint
import re
from llm_config.user_config import UserConfig
import requests


# Global Initialization
config = UserConfig()
# dashscope.api_key = config.dashscope_api_key

class ChatLLMNode(Node):
    def __init__(self):
        super().__init__("ChatLLM_node")
        # 创建行为树的服务端
        self.srv = self.create_service(BehavioursTree, "/llm_input", self.llm_srv_callback)
        self.add_child_tree = self.create_client(ChildTree,"/add_child_tree")

    def handle_dify_stream_response(self,response):
        '''
        处理dify服务返回的流数据
        '''
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
                            if event_type == "message":
                                chunk = event_data.get("answer", "")
                                full_text += chunk
                                # self.get_logger().info(f"收到分块: {chunk}")
                            # elif event_type == "workflow_finished":
                            #     top_data = event_data.get("data", {})
                            #     output = top_data.get("outputs", {})
                            #     chunk = output.get("answer", "")
                            #     full_text += chunk
                                # self.get_logger().info(f"收到分块: {chunk}")
                            elif event_type == "agent_message":
                                chunk = event_data.get("answer", "")
                                full_text += chunk
                                # self.get_logger().info(f"收到分块: {chunk}")
                            
                            # 捕获结束事件及元数据
                            # elif event_type == "message_end":
                            #     metadata = event_data.get("metadata", {})
                            #     self.get_logger().info(f"最终元数据: {metadata}")
                        
                        except json.JSONDecodeError:
                            self.get_logger().error(f"JSON解析失败: {decoded_line}")
        else:
            self.get_logger().error(f"请求失败，状态码: {response.status_code}")
            return False

        # 返回完整结果
        return {
            "content": full_text,
            # "metadata": metadata
        }

    def upload_photo(self,photo_path,dify_key):
        '''
        上传照片到dify服务
        '''
        url = "http://192.168.100.244/v1/files/upload"
        headers = {
            "Authorization": dify_key  # 认证信息
        }
        # 1. 以二进制模式打开文件，并指定 MIME 类型
        with open(photo_path, 'rb') as file:
            # 2. 构建 files 字典：键名必须为 'file'，值包含（文件名, 文件对象, MIME类型）
            files = {'file': ('image', file, 'image/png')}  # MIME 类型需匹配文件格式[4,6](@ref)
            
            # 3. 非文件参数（如 user）通过 data 传递
            data = {'user': 'leo'}
            
            # 4. 发送 POST 请求（移除 json 和 stream 参数）
            response = requests.post(url, headers=headers, files=files, data=data)

        photo_id = response.json()["id"]
        self.get_logger().info(f"图片上传id: {photo_id}")

        return photo_id

    def generate_llm_response(self, messages_input):
        '''
        根据提供的输入消息生成思维链的响应。
        '''
        
        # 调用dify服务-思维链服务
        dify_key = "Bearer app-in7itSS6MVPXDpZNTzHQeT5j"  # 思维链
        url = "http://192.168.100.244/v1/chat-messages" 
        headers = {
            "Authorization": dify_key,  
            "Content-Type": "application/json"  
        }
        data = {
            "inputs": {},
            "query": str(messages_input),
            "response_mode":"streaming",
            "user":"leo",
        }

        # 请求dify服务
        response = requests.post(url, headers=headers,json=data,stream=True)
        
        # 针对streaming格式处理
        result = self.handle_dify_stream_response(response)

        if result:
            # self.get_logger().info(f"\033[36m Qwen响应 : \n{result['content']}\033[0m")
            return result['content']
        else:
            return None

    def get_response_classification(self, llm_response):
        '''
        从ChatLLM的响应中提取响应信息。
        return:
        response_type: 响应类型
        response_data: 响应内容
        response_description: 响应描述
        ''' 

        # 1. 提取响应内容，初始化标签
        chunk = llm_response 
        # 初始化消息文本，默认为None
        response_content = None; response_type = None ; response_data = None ; response_description = None
        self.get_logger().info(f"\033[36m 原始响应: {chunk}\033[0m")

        # 2.1  处理包含 <think> 标签的情况
        start_tag = "<think>"
        end_tag = "</think>"
        if start_tag in chunk and end_tag in chunk:
            chunk = re.sub(r'<think>.*?</think>', '', chunk, flags=re.DOTALL).strip()
        
        if not chunk.strip().startswith('{'):
            chunk = '{' + chunk + '}'

        try:
            # 2.2 解析JSON响应,提取响应类型、响应内容、响应描述
            response_content = json.loads(chunk)
            response_type = response_content["response_type"]
            response_data = response_content["response_data"]
            response_description = response_content["response_description"]
            self.get_logger().info(f"\033[36m 检测到响应类型: {response_type}\033[0m")
            self.get_logger().info(f"\033[36m 检测到响应数据: {response_data}\033[0m")
            self.get_logger().info(f"\033[36m 检测到响应描述: {response_description}\033[0m")
        except (KeyError, json.JSONDecodeError) as e:
            self.get_logger().error(f"响应解析失败: {str(e)}")


        return response_type, response_data, response_description

    def handle_info(self,info):
        '''
        处理info类型响应
        '''
        request = ChildTree.Request()
        request.server_name = "audio_output"
        request.server_type = "BehavioursTree"
        request.server_parameters = info
        self.add_child_tree.call_async(request)

    def handle_task_list(self,task_list):
        '''
        处理任务列表
        '''
        server_name = None ; server_type = None ; server_parameters = None
        self.get_logger().info(f"📝开始发布任务列表")
        for task in task_list:
            server_name = task["server_name"]
            server_type = task["server_type"]
            server_parameters = task["server_parameters"]

            request = ChildTree.Request()
            request.server_name = server_name
            request.server_type = server_type
            request.server_parameters = server_parameters
            self.add_child_tree.call_async(request)
            
            # self.get_logger().info(f"任务: {server_name} {server_type} {server_parameters}")

    def llm_srv_callback(self, request, response):
        '''
        llm_srv_callback函数在ChatLLM服务被调用时触发。
        该函数是ChatLLM节点的主要功能,用于处理输入消息并生成LLM响应。
        '''
        try:
            # 1. 显示收到的输入消息
            self.get_logger().info(f"\033[32m 收到的输入消息: {request.input_data}\033[0m")

            # 2. 将用户消息添加到聊天历史记录中,根据对话历史生成聊天回复
            user_prompt = request.input_data

            # 3. 生成LLM响应
            llm_response = self.generate_llm_response(user_prompt)

            if llm_response: # 检查是否生成了响应
                # 4. 提取响应的相关信息 (响应类型、响应内容、响应描述)
                response_type, response_data, response_description = self.get_response_classification(llm_response)
                # 5. 根据响应类型分类处理
                if response_type == "info":
                    self.get_logger().info(f"INFO处理完成: {response_description}...")
                    self.handle_info(response_description)
                elif response_type == "demo":
                    self.get_logger().info(f"DEMO执行: {response_description}...")
                elif response_type == "task_list":
                    self.get_logger().info(f"TASK LIST: {response_description}...")
                    self.handle_task_list(response_data)
                else:
                    self.get_logger().error(f"未知响应类型: {response_type}")
                
                # 返回响应
                response.success = True
                response.output_data = str(response_description)
                self.get_logger().info(f"LLM processing success: {response_description}")
                return response
            else:
                response.success = False
                return response
        except Exception as e:
            self.get_logger().error(f"Service call exception: {str(e)}")
            response.success = False
            return response


def main(args=None):
    rclpy.init(args=args)
    ChatLLM = ChatLLMNode()
    rclpy.spin(ChatLLM)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


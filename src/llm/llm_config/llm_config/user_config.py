#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2025 Huohaijie @AUBO Robotics
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
# This is a configuration file for a conversational AI assistant
# that uses the llm API for generating responses.
#
# The user can specify the llm language model to be used, the organization
# under which their API key is registered (if applicable), and several parameters
# that affect the creativity and coherence of the AI's responses, such as the
# temperature, top probability cutoff, and frequency and presence penalties.
#
# The user can also specify the prompt given to the AI, the prefix for the AI's response,
# and the maximum number of tokens and length allowed in the response.
#
# The chat history can be stored in a JSON file, with a maximum length limit.
#
# The assistant's behavior can be customized using a RobotBehavior object
# and a list of robot functions.
#
# The API credentials for Amazon AWS are provided, along with parameters
# for AWS S3, Transcribe, and Polly services, and parameters for audio recording.
#
# Author:   howe12@126 Robotics

import os
from ament_index_python.packages import get_package_share_directory



class UserConfig:
    def __init__(self):
        # Qwen API related
        # [required]: DashScope API key
        self.dashscope_api_key = os.getenv("DASHSCOPE_API_KEY")
        self.dashvector_api_key = os.getenv("DASHVECTOR_API_KEY")
        self.dashvector_endpoint = os.getenv("DASHVECTOR_ENDPOINT")
        # [required]: Name of the Qwen language model to be used
        self.qwen_llm_model = "qwen-turbo"
        self.qwen_vlm_model = "qwen-vl-plus"

        # [optional]: Controls the creativity of the AI’s responses. Higher values lead to more creative, but less coherent, responses
        self.llm_temperature = 1
        # [optional]: Probability distribution cutoff for generating responses
        self.llm_top_p = 1
        self.llm_top_k = 50
        # [optional]: Number of responses to generate in batch
        self.llm_n = 1
        # [optional]: Whether to stream response results or not
        self.llm_stream = False
        # [optional]: String that if present in the AI's response, marks the end of the response
        self.llm_stop = "NULL"
        # [optional]: Maximum number of tokens allowed in the AI's respons
        self.llm_max_tokens = 4000
        # self.llm_max_tokens= 16000
        # [optional]: Value that promotes the AI to generates responses with higher diversity
        self.llm_frequency_penalty = 0
        # [optional]: Value that promotes the AI to generates responses with more information at the text prompt
        self.llm_presence_penalty = 0

        # IO related
        # [optional]: The prompt given to the AI, provided by the user
        self.user_prompt = ""
        # [optional]: The generated prompt by the administrator, used as a prefix for the AI's response
        self.system_prompt = self.load_resource_file("llm_config", "llm_prompt.md")
        # [optional]: The generated response provided by the AI
        self.assistant_response = ""

        # Chat history related
        # [optional]: The chat history, including the user prompt, system prompt, and assistant response
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
        # [optional]: The path to the chat history JSON file
        self.chat_history_path = os.path.expanduser("~")
        # [optional]: The limit of the chat history length
        self.chat_history_max_length=16000
        
        # Realsense ROS related
        self.rs_color_image_topic = '/camera/camera/color/image_raw'
        self.rs_depth_image_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.rs_camera_info_topic = '/camera/camera/color/camera_info'

        self.duration = 15
        # [optional]: Audio recording sample rate, in Hz
        self.sample_rate = 16000
        self.vad_threshold = 0.5
        # [optional]: Audio recording gain multiplier
        # Change this to increase or decrease the volume
        self.volume_gain_multiplier = 1
    def load_resource_file(self,package_name, resource_path):
        package_dir = get_package_share_directory(package_name)
        resource_file_path = os.path.join(package_dir, 'resource', resource_path)
        with open(resource_file_path, 'r', encoding='utf-8') as file:
            return file.read()
if __name__ == '__main__':
    config = UserConfig()
    print(config.system_prompt)
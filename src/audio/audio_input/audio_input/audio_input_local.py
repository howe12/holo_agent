#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2025 Huohaijie @NXROBO Robotics
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
# Description: 此脚本实现了一个ROS2节点，用于本地音频输入的录制和语音识别
#
# Node test Method:
# ros2 run audio_input llm_audio_input_local
# ros2 topic echo /audio_input_audio_to_text
# ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
#
# Author:  Huohaijie @NXROBO Robotics


import os
# from modelscope.outputs import OutputKeys
# from modelscope.pipelines import pipeline
# from modelscope.utils.constant import Tasks

# Audio recording related
import sounddevice as sd
import signal
import numpy as np
import pyaudio
import collections
import time
import sys
from array import array
from audio_input.vad import VoiceActivityDetection
from scipy.io.wavfile import write
# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import wave
from threading import Event

import sys
import os

from funasr import AutoModel
from funasr.utils.postprocess_utils import rich_transcription_postprocess

# Global Initialization
# from llm_config.user_config import UserConfig

# 加载用户配置
# config = UserConfig()

class AudioInput(Node):
    def __init__(self):
        """
        初始化AudioInput节点，设置临时音频文件路径、VAD模型路径、提示音路径，
        创建ROS2发布者、订阅者，初始化语音识别管道，读取唤醒词。
        """
        super().__init__("llm_audio_input")

        # 临时音频文件路径
        self.tmp_audio_file = os.path.join(get_package_share_directory("audio_input"),"resource","user_audio_input.wav")
        # 获取语音识别模型文件路径
        self.model_dir = "iic/speech_seaco_paraformer_large_asr_nat-zh-cn-16k-common-vocab8404-pytorch"
        # 获取VAD模型文件路径
        self.vad_model_path = os.path.join(get_package_share_directory("audio_input"), "resource", "silero_vad.onnx")
        # 获取提示音文件路径
        self.ding_sound_path = os.path.join(get_package_share_directory("audio_input"), "resource", "ding-126626.mp3")
        # 初始化语音活动检测模型
        self.vad = VoiceActivityDetection(model_path=self.vad_model_path)

        # 初始化状态发布者，发布节点初始化状态
        self.initialization_publisher = self.create_publisher( String, "/llm_initialization_state", 0)
        # LLM状态发布者，发布LLM的当前状态
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)
        # LLM状态订阅者，监听LLM的状态变化
        self.llm_state_subscriber = self.create_subscription(String, "/llm_state", self.state_listener_callback, 0)
        # 音频转文本结果发布者，发布音频转文本的结果
        self.audio_to_text_publisher = self.create_publisher(String, "/audio_input_audio_to_text", 0)
        # 发布节点初始化完成状态
        self.publish_string("llm_audio_input", self.initialization_publisher)

        # 初始化语音识别模型
        self.model = AutoModel(
            model=self.model_dir,
            vad_model="fsmn-vad",
            vad_kwargs={"max_single_segment_time": 30000},
            device="cuda:0",
        )

        # # 初始化语音识别管道
        # self.inference_pipeline  = pipeline(task=Tasks.auto_speech_recognition,
        #               model='iic/speech_paraformer-large-contextual_asr_nat-zh-cn-16k-common-vocab8404',
        #               model_revision="v2.0.4",
        #               punc_model='iic/punc_ct-transformer_zh-cn-common-vocab272727-pytorch',
        #               punc_model_revision="v2.0.4")
        # 读取唤醒词
        self.hotword = self.read_hotword()

    def read_hotword(self):
        """
        从资源文件中读取唤醒词。

        Returns:
            str: 读取到的唤醒词。
        """
        package_dir = get_package_share_directory("audio_input")
        resource_file_path = os.path.join(package_dir, 'resource', 'hotword.txt')
        with open(resource_file_path, 'r', encoding='utf-8') as file:
            return file.readline()

    def listen(self):
        """
        监听音频输入，使用语音活动检测（VAD）检测语音的开始和结束，
        录制音频并返回音频数据。

        Returns:
            bytes: 录制的音频数据，如果没有录制到有效音频则返回False。
        """
        #################################################
        ###             1. 配置PyAudio参数             ###
        #################################################
        # 音频格式
        FORMAT = pyaudio.paInt16
        # 音频通道数
        CHANNELS = 1
        # 采样率
        RATE = 16000   # config.sample_rate
        # 音频块大小，单位为毫秒
        CHUNK_SIZE = 100  # ms
        # 用于判断语音活动的填充时长，单位为毫秒
        PADDING_DURATION_MS = 400  # 400 ms for judgement
        # 每次读取的音频块大小
        RAW_CHUNK = int(RATE * CHUNK_SIZE / 1000)  # Chunk size to read
        # 开始检测所需的窗口块数量
        NUM_WINDOW_CHUNKS = int(PADDING_DURATION_MS / CHUNK_SIZE)
        # 结束检测所需的窗口块数量
        NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS * 2
        # 初始化PyAudio对象
        pa = pyaudio.PyAudio()
        self.exit_event = Event()

        #################################################
        ###             2. PyAudio打开音频输入          ###
        #################################################  
        # 打开音频输入流
        self.stream = pa.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, start=False, frames_per_buffer=RAW_CHUNK)
        # 是否获取到完整句子的标志
        self.got_a_sentence = False
        # 是否退出监听的标志
        self.leave = False

        #################################################
        ###             3. 判断系统中断信号              ###
        #################################################  
        def handle_int(sig, frame):
            """专业级信号处理函数"""
            self.get_logger().warn(f"接收到中断信号: {signal.Signals(sig).name}")
            self.leave = True
            self.got_a_sentence = True
            self.exit_event.set()
            
            # 立即中断音频流（关键操作）
            if self.stream.is_active():
                try:
                    self.stream.stop_stream()
                except Exception as e:
                    self.get_logger().error(f"流中断异常: {e}")

        # 注册中断信号处理函数
        # signal.signal(signal.SIGINT, handle_int)

        #################################################
        ###             4. 设置环形缓冲区存储音频块       ###
        #################################################  
        while not self.leave:
            # 语音开始的标志
            triggered = False
            # 环形缓冲区，用于存储音频块
            ring_buffer = collections.deque(maxlen=NUM_WINDOW_CHUNKS)
            # 存储语音帧的列表
            voiced_frames = []
            # 开始检测的环形缓冲区标志
            ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
            # 开始检测的环形缓冲区索引
            ring_buffer_index = 0
            # 结束检测的环形缓冲区标志
            ring_buffer_flags_end = [0] * NUM_WINDOW_CHUNKS_END
            # 结束检测的环形缓冲区索引
            ring_buffer_index_end = 0
            # 存储原始音频数据的数组
            raw_data = array('h')   
            # 音频数据索引
            index = 0
            # 记录开始时间
            StartTime = time.time()
            # 播放提示音
            os.system(f"mpv {self.ding_sound_path}")

            #################################################
            ###             5. 启动音频流                   ###
            #################################################  
            # 记录开始录音日志
            self.get_logger().info("Start local recording... ")
            # 启动音频流
            self.stream.start_stream()

            #################################################
            ###             6. 循环读取音频块               ###
            #################################################  
            while not self.got_a_sentence and not self.leave:
                # 读取音频块
                chunk = self.stream.read(RAW_CHUNK)
                # 将音频块添加到原始数据数组
                raw_data.extend(array('h', chunk))
                index += RAW_CHUNK
                # 计算已使用的时间
                TimeUse = time.time() - StartTime

                #################################################
                ###             6.1 检测语音活动                ###
                #################################################  
                # 将音频块转换为NumPy数组
                frame_np = np.frombuffer(chunk, dtype=np.int16)
                # 归一化音频数据
                frame_float = frame_np / np.iinfo(np.int16).max  
                # 使用VAD模型进行语音活动检测
                res = self.vad(frame_float, sr=RATE).item()

                #################################################
                ###             6.2  更新环形缓冲区             ###
                #################################################  
                # 判断是否有语音活动
                active = res > 0.5  # config.vad_threshold
                # 输出语音活动状态
                sys.stdout.write('1' if active else '_')
                # 更新开始检测的环形缓冲区标志
                ring_buffer_flags[ring_buffer_index] = 1 if active else 0
                ring_buffer_index += 1
                ring_buffer_index %= NUM_WINDOW_CHUNKS

                # 更新结束检测的环形缓冲区标志
                ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
                ring_buffer_index_end += 1
                ring_buffer_index_end %= NUM_WINDOW_CHUNKS_END

                #################################################
                ###             6.3  短窗口检测开始             ###
                #################################################                 
                # 开始点检测
                if not triggered:
                    if TimeUse > 5:
                        # 监听超时，记录日志并返回
                        self.get_logger().info('end of listening')
                        return False
                    # 将音频块添加到环形缓冲区
                    ring_buffer.append(chunk)
                    # 计算语音活动的块数量
                    num_voiced = sum(ring_buffer_flags)
                    if num_voiced > 0.8 * NUM_WINDOW_CHUNKS:
                        # 达到语音开始条件，标记为已触发
                        sys.stdout.write(' Open ')
                        triggered = True
                        # 将环形缓冲区中的音频块添加到语音帧列表
                        voiced_frames.extend(ring_buffer)
                        # 清空环形缓冲区
                        ring_buffer.clear()

                #################################################
                ###             6.4  长窗口检测结束             ###
                #################################################  
                # 结束点检测
                else:
                    # 将音频块添加到语音帧列表
                    voiced_frames.append(chunk)
                    # 将音频块添加到环形缓冲区
                    ring_buffer.append(chunk)

                    # 计算非语音活动的块数量
                    num_unvoiced = NUM_WINDOW_CHUNKS_END - sum(ring_buffer_flags_end)
                    if num_unvoiced > 0.80 * NUM_WINDOW_CHUNKS_END or TimeUse > 15: # config.duration，两者情况结束录制:
                        # 达到语音结束条件，播放提示音，标记为未触发，获取到完整句子
                        sys.stdout.write(' Close ')
                        os.system(f"mpv {self.ding_sound_path}")
                        triggered = False
                        self.got_a_sentence = True

                # 刷新标准输出
                sys.stdout.flush()

            #################################################
            ###             7. 保存音频，停止音频流          ###
            #################################################  
            # 换行
            sys.stdout.write('\n')
            if not voiced_frames:
                # 没有录制到有效音频，返回False
                return False

            # 保存wav文件
            wf = wave.open(self.tmp_audio_file, "wb")
            wf.setnchannels(CHANNELS)
            STANDARD_WIDTH = 2  # 16-bit音频固定使用2字节
            # wf.setsampwidth(pa.get_sample_size(FORMAT))
            wf.setsampwidth(STANDARD_WIDTH)
            wf.setframerate(RATE)
            wf.writeframes(b"".join(voiced_frames))
            wf.close()

            # 将语音帧列表合并为二进制数据
            data = b''.join(voiced_frames)

            # 停止音频流
            self.stream.stop_stream()
            # 记录停止录音日志
            self.get_logger().info("Stop local recording!")
            # 重置获取句子标志
            self.got_a_sentence = False
            # 标记为退出监听
            self.leave = True

        #################################################
        ###             8. 关闭音频流，输出音频数据       ###
        #################################################  
        # 关闭音频流
        self.stream.close()
        return data


    def state_listener_callback(self, msg):
        """
        处理接收到的LLM状态消息，当状态为 "listening" 时，开始监听音频输入。

        Args:
            msg (std_msgs.msg.String): 接收到的状态消息。
        """

        # 检查接收到的消息数据是否为 "listening"
        if msg.data == "listening":
            # 记录当前状态信息到日志
            self.get_logger().info(f"STATE: {msg.data}")
            # 当状态为 "listening" 时，调用 action_function_listening 方法开始监听音频输入
            self.action_function_listening()

    def action_function_listening(self):
        """
        开始监听音频输入，将录制的音频进行语音识别，
        并将识别结果发布到ROS2话题。
        """
        
        # 调用listen方法录制音频
        audio_data = self.listen()
        # 发布状态为 "input_processing"
        self.publish_string("input_processing", self.llm_state_publisher)

        if audio_data:
            # 记录开始转换时间
            start = time.time()
            # 记录开始转换日志
            self.get_logger().info("Local Converting...")
            # 进行语音识别
            # stt_result = self.inference_pipeline(audio_data, hotword=self.hotword)

            stt_result = self.model.generate(
                # input=f"/home/leo/sagittarius_humble_ws/src/Sag_Audio/audio_input/resource/who.wav",
                input=audio_data,
                cache={},
                language="zn",  # "zn", "en", "yue", "ja", "ko", "nospeech"
                use_itn=True,
                batch_size_s=60,
                merge_vad=True,  #
                merge_length_s=15,
            )

            # 记录结束转换时间
            end = time.time()
            # 记录语音识别耗时日志
            self.get_logger().info(f'paraformer spent: {end - start} s')

            # 获取识别结果文本
            transcript_text = stt_result[0]["text"]
            # 记录音频转文本完成日志
            self.get_logger().info("Audio to text conversion complete!")

            # Step 8: Publish the transcribed text to ROS2
            if transcript_text == "":  # Empty input
                self.get_logger().info("Empty input!")
                self.publish_string("listening", self.llm_state_publisher)
            else:
                self.publish_string(transcript_text, self.audio_to_text_publisher)

    def publish_string(self, string_to_send, publisher_to_use):
        """
        发布字符串消息到指定的ROS2发布者。

        Args:
            string_to_send (str): 要发布的字符串消息。
            publisher_to_use (rclpy.publisher.Publisher): 要使用的ROS2发布者。
        """
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )


def main(args=None):
    """
    主函数，初始化ROS2节点，启动节点，最后销毁节点并关闭ROS2。
    """
    # 初始化ROS2
    rclpy.init(args=args)

    # 创建AudioInput节点实例
    audio_input = AudioInput()

    # 启动节点
    rclpy.spin(audio_input)

    # 销毁节点
    audio_input.destroy_node()
    # 关闭ROS2
    rclpy.shutdown()


if __name__ == "__main__":
    main()

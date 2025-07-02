#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from modelscope.outputs import OutputKeys
from modelscope.pipelines import pipeline
from modelscope.utils.constant import Tasks

from llm_interfaces.srv import BehavioursTree
import pyaudio
import wave


class AudioOutput(Node):
    def __init__(self):
        """
        初始化AudioOutput节点
        """
        super().__init__("audio_output")

        # 临时音频文件路径
        self.tmp_wav_file = os.path.join(get_package_share_directory("audio_output"),"resource","user_audio_output.wav")
        # 获取语音合成模型文件路径
        # self.model_dir = "iic/speech_sambert-hifigan_tts_zh-cn_16k"
        self.model_dir = "/home/leo/.cache/modelscope/hub/models/iic/speech_sambert-hifigan_tts_zh-cn_16k"
        # 初始化语音合成模型
        self.sambert_hifigan_tts = pipeline(task=Tasks.text_to_speech, model=self.model_dir)
        # 初始化PY音频处理库
        self.p = pyaudio.PyAudio()
        # 语音合成订阅者，监听语音合成请求
        # self.audio_output_subscriber = self.create_subscription(String, "/audio_output_content", self.audio_output_callback, 0)
        # 创建音频输出识别服务
        self.srv = self.create_service(BehavioursTree, "/audio_output", self.audio_output_srv)
        # LLM状态发布者，发布LLM的当前状态
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)

    def play_wav(self):
        """
        波频播放函数，读取并播放WAV文件。
        Args:
            None
        Returns:
            None
        """
        # 打开WAV文件
        wf = wave.open(self.tmp_wav_file, "rb")

        # 打开播放流
        stream = self.p.open(format=self.p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)

        # 流式播放
        data = wf.readframes(1024)
        while data:
            stream.write(data)
            data = wf.readframes(1024)

        # 清理资源
        stream.close()
        # self.p.terminate()

    def audio_output_srv(self,request, response):
        """
        语音合成回调函数，接收语音合成请求，进行语音合成并播放。
        Args:
            request.input_data (String): 语音合成请求消息，包含要合成的文本。
        Returns:
            None
        """
        # 语音合成
        output = self.sambert_hifigan_tts(input=request.input_data,voice='zhizhe_emo')
        # 保存音频文件
        wav = output[OutputKeys.OUTPUT_WAV]
        with open(self.tmp_wav_file, 'wb') as f:
            f.write(wav)
        # 播放音频文件
        self.play_wav()
        response.success = True
        response.output_data = "success"
        return response


def main(args=None):
    """
    主函数，初始化ROS2节点，启动节点，最后销毁节点并关闭ROS2。
    """
    # 初始化ROS2
    rclpy.init(args=args)

    # 创建AudioOutput节点实例
    audio_output = AudioOutput()

    # 启动节点
    rclpy.spin(audio_output)

    # 销毁节点
    audio_output.destroy_node()
    # 关闭ROS2
    rclpy.shutdown()


if __name__ == "__main__":
    main()

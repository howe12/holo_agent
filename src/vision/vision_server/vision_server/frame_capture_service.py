#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from llm_interfaces.srv import ChatLLM,BehavioursTree
import os
import cv2
from cv_bridge import CvBridge
import threading
from ament_index_python.packages import get_package_share_directory


class FrameCaptureService(Node):
    def __init__(self):
        super().__init__('frame_capture_service')
        
        # 共享变量
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.bridge = CvBridge()
        vision_server = get_package_share_directory('vision_server')  # 功能包地址
        self.resource_dir= os.path.join(vision_server, 'resource')    # 放置照片位置

        # 创建订阅器
        self.subscription = self.create_subscription(Image,'camera/camera/color/image_raw',self.image_callback,10)
         
        # 创建服务
        self.srv = self.create_service(BehavioursTree, 'capture_frame',self.capture_callback)
        
        self.get_logger().info('Frame capture service ready')

    def image_callback(self, msg):
        """异步接收图像更新"""
        try:
            with self.frame_lock:
                # 将ROS2图像消息转换为OpenCV格式
                self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {str(e)}')

    def save_image_to_resource(self, image):
        """保存图像到资源文件夹"""
        filename = f"vlm.png"
        save_path = os.path.join(self.resource_dir, filename)
        
        # 保存图像
        cv2.imwrite(save_path, image)
        self.get_logger().info(f'Image saved to: {save_path}')
        
        return save_path

    def capture_callback(self, request, response):
        """服务调用处理"""
        with self.frame_lock:
            if self.latest_frame is None:
                response.success = False
                self.get_logger().warn('No frame available for capture')
                return response
            try:
                self.save_image_to_resource(self.latest_frame)
                response.success = True
                self.get_logger().info('Frame captured and processed')
            except Exception as e:
                response.success = False
                self.get_logger().error(f'Frame conversion failed: {str(e)}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = FrameCaptureService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import time

import numpy as np
import tf2_ros
import tf_transformations
# from time import time

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError

import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class YoloDetection(Node):
# 初始化函数，包括加载模型和创建订阅者
    def __init__(self):
        # 初始化节点
        super().__init__('Yolo')
        self.logger = get_logger("yolov8") # 创建日志记录器

        # 初始化模型信息
        ros2_yolov8 = get_package_share_directory('ros2_yolov8')  # 功能包地址
        model_path = os.path.join(ros2_yolov8, 'config', 'meal2.onnx')  # 模型路径
        self.model = YOLO(model_path) # 加载模型
        self.device = 'cpu'
        self.conf = 0.5  # 设置模型置信度

        # 初始化相机信息
        self.image_topic = self.declare_parameter('image_topic', '/camera1/image_raw').value
        self.image_info_topic = self.declare_parameter('image_info_topic', '/camera1/camera_info').value
        self.sub_info = self.create_subscription(CameraInfo, self.image_info_topic, self.imageCameraInfoCallback, 10)
        time.sleep(3)
        self.image_subscriber = self.create_subscription(Image,  self.image_topic, self.detect, 10)
                                 

        self.detect_img_pub = self.create_publisher(Image, "/detect_image", 1)    # 最终图像输出话题
        self.camera_reference_frame = "sgr532/usb_cam_link"                         # 定义摄像头坐标系
        self.tf_pub = tf2_ros.TransformBroadcaster(self)                    # 物体坐标变换广播
        self.obj_pub = self.create_publisher(String, "/object_id", 10)
 
        self.bridge = CvBridge()
        self.depth_z = 0.25

        

    def imageCameraInfoCallback(self, msg):
        """
        获取相机内参信息
        """
        # 获取图像尺寸
        self.image_width = msg.width
        self.image_height = msg.height
        # 获取相机内参
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def predict(self, frame):
        # 使用模型进行预测
        results = self.model(frame, conf=self.conf,show_labels=False,show_conf=False,show_boxes=False)
        return results

    def plot_boxes(self, results, frame):
        '''计算帧率和绘制边界框'''
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        frame = results[0].plot()
        return frame

    def detect(self, msg):
        '''检测函数'''
        # self.logger.info(f"正在")
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # 将ROS消息转换为OpenCV格式的图像
        results    = self.predict(self.color_image)             # yolov8检测
        frame      = self.plot_boxes(results, self.color_image) # 绘制边界框
        detect_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")   # 将OpenCV格式的图像转换为ROS消息
        self.detect_img_pub.publish(detect_img)                 # 发布检测图像

        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            xywh = boxes.xywh
            size = int(xywh.size()[0]) # 获取张量的行数
            i = 0
            for i in range(size):  
                clsaa_id = int(boxes.data[i][5]) # 获取物体id
                print("clsaa_id",clsaa_id)

                m_x = int(xywh[i][0])+int(xywh[i][2]/2) # 获取物体中心点位置
                m_y = int(xywh[i][1])+int(xywh[i][3]/2)
                self.logger.info(f"🎯 中心点x,y: [{m_x},{m_y}]")

                x = (m_x - self.cx) / self.fx
                y = (m_y - self.cy) / self.fy
                point_x = self.depth_z * x
                point_y = self.depth_z * y
                point_z = self.depth_z

                # 创建TF变换消息并发布
                cube_tf = TransformStamped()
                cube_tf.header.stamp = self.get_clock().now().to_msg()
                cube_tf.header.frame_id = self.camera_reference_frame
                cube_tf.child_frame_id = "object_"+str(clsaa_id)
                cube_tf.transform.translation.x = float(point_z)                # 将Y轴坐标赋值给X轴
                cube_tf.transform.translation.y = -float(point_x)               # 将X轴坐标赋值给Y轴
                cube_tf.transform.translation.z = -float(point_y)               # 将Z轴坐标赋值给Z轴
                ori = tf_transformations.quaternion_from_euler(0, 0, 0)  # 计算姿态的四元数
                cube_tf.transform.rotation.x = ori[0]
                cube_tf.transform.rotation.y = ori[1]
                cube_tf.transform.rotation.z = ori[2]
                cube_tf.transform.rotation.w = ori[3]
                msg = String()
                msg.data = str(clsaa_id)
                self.obj_pub.publish(msg)
                self.tf_pub.sendTransform(cube_tf)  # 发布TF变换消息


def main():
    rclpy.init()
    node = YoloDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

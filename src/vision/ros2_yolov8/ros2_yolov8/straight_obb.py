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
        model_path = os.path.join(ros2_yolov8, 'config', 'best.pt')  # 模型路径
        self.model = YOLO(model_path) # 加载模型

        # 初始化相机信息
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.imageCameraInfoCallback, 10)    # 相机信息话题
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.detect, 10)  
        time.sleep(3)                         # rgb图像话题
        self.depth_sub = self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.camera_depth_cb, 10) # 深度图像话题
        self.debug_pub = self.create_publisher(Image, "/debug_image", 1)    # 最终图像输出话题
        self.camera_reference_frame = "camera_link"                         # 定义摄像头坐标系
        self.tf_pub = tf2_ros.TransformBroadcaster(self)                    # 物体坐标变换广播
        self.obj_pub = self.create_publisher(String, "/object_id", 10)
 
        # time.sleep(5)
        
        self.bridge = CvBridge()
        self.device = 'cpu'
        self.conf = 0.5  # 设置模型置信度
        
        self.obj_id = [1,2,3,4]

        

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

        for result in results:
            # 输出结果
            # self.logger.info(f"🎯 结果cls为: \n{result.obb.cls}")
            # # self.logger.info(f"🎯 第二个物体为: \n{result.obb.cls[1]}")
            # self.logger.info(f"🎯 结果size为: \n{result.obb.cls.size(0)}")
            # self.logger.info(f"🎯 xywhr为: \n{result.obb.xywhr}")
            # self.logger.info(f"🎯 结果x为: \n{result.obb.xywhr[0,0]}")
            # self.logger.info(f"🎯 结果y为: \n{result.obb.xywhr[0,1]}")
            # self.logger.info(f"🎯 结果xyxyxyxy为: \n{result.obb.xyxyxyxy}")
            # # 左上角坐标
            # self.logger.info(f"🎯 结果x1为: \n{result.obb.xyxyxyxy[0,0]}")
            # # 右下角坐标
            # self.logger.info(f"🎯 结果x2为: \n{result.obb.xyxyxyxy[0,2]}")
            
            self.obb = result.obb



            # # 计算ROI的边界
            # x1, y1 = np.min(points, axis=0)  # 左上角
            # x2, y2 = np.max(points, axis=0)  # 右下角
            # roi = frame[y1:y2, x1:x2]

            # # 创建黑色掩码
            # mask = np.zeros_like(roi, dtype=np.uint8)
            # # 在掩码上画一个白色圆形
            # cv2.circle(mask, (int(result.obb.xywhr[0,0]), int(result.obb.xywhr[0,1])), 10, (255, 255, 255), -1)
            # # 使用 bitwise_and 仅保留圆形部分
            # roi_circle = cv2.bitwise_and(roi, mask)


        return results

    def plot_boxes(self, results, frame):
        # 计算帧率和绘制边界框

        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        frame = results[0].plot()
        # # 将物体中心点在图片上显示出来
        # cv2.circle(frame, (int(self.obb.xywhr[0,0]), int(self.obb.xywhr[0,1])), 3, (0, 255, 0), 5)
        # # 添加一条线，从物体中心点，斜率为物体的角度的线
        
        # line_length = 30   # 线段长度
        # angle = float(self.obb.xywhr[0, 4])  # 线段角度（弧度）

        # # 计算终点坐标
        # end_x = int(int(self.obb.xywhr[0,0]) + line_length * np.cos(angle))
        # end_y = int(int(self.obb.xywhr[0,1]) + line_length * np.sin(angle))

        # cv2.line(frame, (int(self.obb.xywhr[0,0]), int(self.obb.xywhr[0,1])), (end_x, end_y), (0, 255, 0), 2)
        
        # cv2.imshow("frame",frame)
        # cv2.waitKey(1)

        return frame

    def detect(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results    = self.predict(self.color_image)
        frame      = self.plot_boxes(results, self.color_image)
        detect_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.debug_pub.publish(detect_img)

    # 深度图像回调函数
    def camera_depth_cb(self, msg):
        
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")     # 将ROS消息转换为OpenCV格式的深度图像
        depth_array = np.array(self.depth_image, dtype=np.float32)     # 将深度图像转换为NumPy数组
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX) # 将深度图像数据归一化到0-1范围
        depth_8 = (depth_array * 255).round().astype(np.uint8)         # 将归一化后的深度图像转换为8位无符号整数
        cv_depth = np.zeros_like(self.color_image)                     # 创建与RGB图像相同大小的深度图像
        try:
            cv_depth[:, :, 0] = depth_8                                # 将深度图像数据复制到RGB图像的每个通道
            cv_depth[:, :, 1] = depth_8
            cv_depth[:, :, 2] = depth_8
        except:
            return   
         
        if len(self.obb) == None:
            return
        
        size = self.obb.cls.size(0)

        # 便利所有的物体

        for i in range(size):
            # 获取物体的边界框坐标
            m_x = int(self.obb.xywhr[i,0])
            m_y = int(self.obb.xywhr[i,1])
            m_w = int(self.obb.xywhr[i,2])
            m_h = int(self.obb.xywhr[i,3])
            m_r = self.obb.xywhr[i,4]
            ip  = int(self.obb.cls[i])
        
            """
            # 获取圆形ROI区域
            center = (m_x, m_y)  # 圆心位置
            radius = int(m_h/2)  # 圆形半径 
            mask = np.zeros_like(depth_array, dtype=np.float32)  # 创建一个和原图大小相同的黑色掩码图像
            cv2.circle(mask, center, radius, 255, -1)            # 在掩码图像中绘制一个白色圆形，-1 表示填充圆形            
            roi_circle = cv2.bitwise_and(depth_array, mask)      # 提取圆形ROI区域，通过按位与操作
            count = 0                                            # 初始化统计深度次数
            sum_z = 0.0                                          # 初始化深度值总和
            # 计算ROI区域内有效像素点数量和深度值总和
            for i in range(0, roi_circle.shape[0]):
                for j in range(0, roi_circle.shape[1]):
                    value = roi_circle.item(i, j) / 1000.0
                    
                    if value > 0.05:
                        self.get_logger().info(f"🎯 第{i,j}像素的value: {value}")
                        count += 1
                        sum_z += value
            if count == 0 or sum_z == 0:
                return 
            
            mean_z = sum_z / count # 计算ROI区域内平均深度值
            print(f"ROI 区域内的平均深度值: {mean_z} 米")
            cv2.imshow("Depth", depth_array)
            cv2.imshow("Mask", mask)
            cv2.imshow("Circular ROI", roi_circle)
            cv2.waitKey(1)
            """

            # 获取矩形ROI区域
            obb_xyxyxyxy = self.obb.xyxyxyxy[0]  # 取出第一个目标的 8 个坐标值      
            points = np.array(obb_xyxyxyxy.view(4, 2).numpy(), dtype=np.int32) # 重新整理成 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
            left_top_x , left_top_y  = np.min(points, axis=0)  # 左上角
            right_low_x, right_low_y = np.max(points, axis=0)  # 右下角
            roi_depth = self.depth_image[left_top_y:right_low_y,left_top_x:right_low_x] # 获取ROI区域的深度图像
            count = 0                               # 初始化统计深度次数
            sum_z = 0.0                             # 初始化深度值总和
            
            # 计算ROI区域内有效像素点数量和深度值总和
            for j in range(0, roi_depth.shape[0]):
                for k in range(0, roi_depth.shape[1]):
                    value = roi_depth.item(j, k) / 1000.0
                    if value > 0:
                        count += 1
                        sum_z += value
            if count == 0 or sum_z == 0:
                return 
            
            mean_z = sum_z / count # 计算ROI区域内平均深度值
            # self.get_logger().info(f"🎯 深度值为: {mean_z}")
        
            # 计算物体在相机坐标系下的三维坐标
            x = (m_x - self.cx) / self.fx
            y = (m_y - self.cy) / self.fy
            point_x = mean_z * x
            point_y = mean_z * y
            point_z = mean_z

            # 创建TF变换消息并发布
            cube_tf = TransformStamped()
            cube_tf.header.stamp = self.get_clock().now().to_msg()
            cube_tf.header.frame_id = self.camera_reference_frame
            cube_tf.child_frame_id = "object_"+str(ip)
            cube_tf.transform.translation.x = float(point_z)                # 将Y轴坐标赋值给X轴
            cube_tf.transform.translation.y = -float(point_x)               # 将X轴坐标赋值给Y轴
            cube_tf.transform.translation.z = -float(point_y)               # 将Z轴坐标赋值给Z轴
            ori = tf_transformations.quaternion_from_euler(m_r+1.57, 0, 0)  # 计算姿态的四元数
            cube_tf.transform.rotation.x = ori[0]
            cube_tf.transform.rotation.y = ori[1]
            cube_tf.transform.rotation.z = ori[2]
            cube_tf.transform.rotation.w = ori[3]
            msg = String()
            msg.data = str(ip)
            self.obj_pub.publish(msg)
            self.tf_pub.sendTransform(cube_tf)  # 发布TF变换消息

            # # 合并RGB图像和深度图像，并发布调试信息
            # rgbd = np.concatenate((self.color_image, cv_depth), axis=1)
            # try:
            #     # faces_message = self.bridge.cv2_to_imgmsg(rgbd, "bgr8")
            #     # self.debug_pub.publish(faces_message)
            #     pass
            # except CvBridgeError as e:
            #     print(e)

            
        
     

def main():
    rclpy.init()
    node = YoloDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetection(Node):
# 
    def __init__(self):
        '''初始化函数，包括加载模型和创建订阅者'''
        super().__init__('Yolo') # 创建订阅者    
        self.logger = get_logger("yolov8") # 创建日志记录器
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.detect, 10) # 订阅图像话题
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') # 加载模型
        self.conf = 0.3  # 设置模型置信度

    def plot_boxes(self, results, frame): 
        """计算帧率和绘制边界框."""
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        frame = results[0].plot()
        return frame

    def detect(self, msg):
        """处理接收到的图像信息."""
        self.logger.info("==== get image ====")
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(self.color_image, conf=self.conf) # 使用模型进行预测
        self.boxes = results[0].boxes.data # 用于边界框输出的Boxes对象
        print("boxes=",self.boxes)
        frame = self.plot_boxes(results, self.color_image) # 计算帧率
        self.boxes = None
        cv2.imshow('Object Detection', frame)
        cv2.waitKey(3)
            
def main():
    rclpy.init()
    node = YoloDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

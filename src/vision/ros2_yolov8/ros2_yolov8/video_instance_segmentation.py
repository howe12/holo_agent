import sys
import cv2
from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n-seg.pt')  # pretrained YOLOv8n model

scripts_path = sys.path[0] # 当前脚本的目录

# 打开视频文件
video_path = scripts_path+'/../pictures/fruit.mp4'
cap = cv2.VideoCapture(video_path)

# 循环检测视频帧
while cap.isOpened():
    print("readying")

    # 读取视频的一帧
    success, frame = cap.read()

    if success:
        # 对帧运行YOLOv8推理
        results = model(frame)

        # 显示results
        print(results)

        # 在帧上显示检测结果
        annotated_frame = results[0].plot()

        # 设置显示窗口的名称和大小
        cv2.namedWindow('My Window', cv2.WINDOW_NORMAL)  # 使用cv2.WINDOW_NORMAL标志可以调整窗口大小
        cv2.resizeWindow('My Window', 800, 600)  # 设置窗口大小为800x600像素

        # 显示带有注释的帧
        cv2.imshow("My Window", annotated_frame)

        # 如果按下'q'键则跳出循环
        if cv2.waitKey(60) & 0xFF == ord("q"):  # 等待60毫秒，如果按下'q'键则跳出循环
            break
    else:
        # 如果到达视频的结尾则跳出循环
        break

# 释放视频捕捉对象并关闭显示窗口
cap.release()
cv2.destroyAllWindows()
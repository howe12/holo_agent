from ultralytics import YOLO
import sys

# Load a model
model = YOLO('yolov8n.pt')  # pretrained YOLOv8n model

scripts_path = sys.path[0] # 当前脚本的目录

# Run batched inference on a list of images
# results = model([scripts_path+'/../pictures/fruit_1.jpg', scripts_path+'/../pictures/fruit_2.jpg'])  # return a list of Results objects
results = model([scripts_path+'/../pictures/two_blue_cube.png'])  # return a list of Results objects


# Process results generator
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs

    xywh = boxes.xywh
    size = int(xywh.size()[0]) # 获取张量的行数
    i = 0
    for i in range(size):  
        
        print("############################")
        clsaa_id = boxes.cls[i] # 获取物体id
        print("clsaa_id",clsaa_id)

        m_x = int(xywh[i][0])+int(xywh[i][2]/2) # 获取物体中心点位置
        m_y = int(xywh[i][1])+int(xywh[i][3]/2)
        print("x:",m_x,"y:",m_y)
        

    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    result.show()  # display to screen
    result.save(filename='result.jpg')  # save to disk
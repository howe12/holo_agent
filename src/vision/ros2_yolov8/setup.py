from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_yolov8'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加配置文件，将config文件夹添加到安装路径下
        (os.path.join('share', package_name,'config'), glob(os.path.join('config', '*'))),
        (os.path.join('lib', package_name, 'config'), glob(os.path.join('config', '*'))),

        # (os.path.join('share', package_name,'config'), glob(os.path.join('config', 'nx_obb_int8_openvino_model','*'))),
        # (os.path.join('lib', package_name, 'config'), glob(os.path.join('config', 'nx_obb_int8_openvino_model','*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='howe12@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_object_detection = ros2_yolov8.ros2_object_detection:main',
            'ros2_instance_segmentation = ros2_yolov8.ros2_instance_segmentation:main',
            'ros2_pose_detection = ros2_yolov8.ros2_pose_detection:main',
            'yolo_predict_tf = ros2_yolov8.yolo_predict_tf:main',
            'fruits_predict_tf = ros2_yolov8.fruits_predict_tf:main',
            'ros2_transportation_detection = ros2_yolov8.ros2_transportation_detection:main',
            'straight_obb = ros2_yolov8.straight_obb:main',
            'meal_predict_tf = ros2_yolov8.meal_predict_tf:main'

        ],
    },
)

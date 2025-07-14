from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools',"llm_config", "llm_interfaces"],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='howe12@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "frame_capture_service = vision_server.frame_capture_service:main",
        ],
    },
)

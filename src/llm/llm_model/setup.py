from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'llm_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob(os.path.join('launch','*launch.py'))),
    ],
    install_requires=['setuptools',"llm_config", "llm_interfaces"],
    zip_safe=True,
    maintainer='haijiehuo',
    maintainer_email='howe12@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "chatllm = llm_model.chatLLM:main",
            "chatllm_bt = llm_model.chatLLM_bt:main",
        ],
    },
)

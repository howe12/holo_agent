from setuptools import setup
from glob import glob
package_name = 'llm_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob('resource/*')),
    ],
    install_requires=["setuptools", "llm_config", "llm_interfaces"],
    zip_safe=True,
    maintainer='aubo',
    maintainer_email='zhd172@gmail.com',
    description='The llm_robot package provides a ChatGPT function call server to simulate function calls for any robot',
    license="Apache-2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motion_robot = llm_robot.motion_tasks_robot:main",
            "arm_robot = llm_robot.arm_robot:main",
        ],
    },
)

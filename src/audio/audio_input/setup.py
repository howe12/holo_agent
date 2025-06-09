from setuptools import setup
from glob import glob
package_name = 'audio_input'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*')),
    ],
    # install_requires=['setuptools', 'llm_config'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='howe',
    maintainer_email='howe12@126.com',
    description='The audio_input package contains input nodes for the ROS-LLM.',
    license="Apache-2.0",
    tests_require=[''],
    entry_points={
        'console_scripts': [
            "audio_input_local.py = audio_input.audio_input_local:main",
        ],
    },
)

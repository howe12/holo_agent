from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'llm_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='howe',
    maintainer_email='howe12@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=[''],
    entry_points={
        'console_scripts': [
        ],
    },
)

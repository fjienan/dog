import os
from glob import glob
from setuptools import setup

package_name = 'laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # 安装config目录
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 安装launch目录
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Laser package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 确保Real.py作为可执行节点注册
            'ros2publisher = laser.ros2publisher:main',
            'Real = laser.Real:main'
        ],
    },
)
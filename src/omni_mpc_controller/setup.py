from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'omni_mpc_controller'

# 自动包含所有 config/* 和 resource/* 文件
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # 添加 config 文件夹中的所有文件
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    # 如果有 launch 文件，也可以添加
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,  # 使用更新后的 data_files
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dell',
    maintainer_email='dell@todo.todo',
    description='Omnidirectional MPC Controller for two robots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omni_mpc_controller_node = omni_mpc_controller.omni_mpc_controller_node:main',
            'MPC = omni_mpc_controller.MPC:main',
            'keyboard_control = omni_mpc_controller.keyboard_control:main'
        ],
    },
)
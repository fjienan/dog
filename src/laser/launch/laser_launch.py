from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取laser包的share目录路径
    pkg_dir = get_package_share_directory('laser')
    
    # 指定参数文件路径
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # 创建节点配置
    laser_node = Node(
        package='laser',          # 包名
        executable='laser_position_1', # 可执行文件名称（在setup.py中定义）
        name='laser_position_1',   # 节点名称
        parameters=[params_file] # 加载参数文件
    )
    
    
    return LaunchDescription([laser_node])
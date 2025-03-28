from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    try:
        # Try to get livox_ros_driver2 package
        livox_ros_driver_share = get_package_share_directory('livox_ros_driver2')
        livox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_ros_driver_share, 'launch_ROS2', 'msg_MID360_launch.py')
            )
        )
        ld.add_action(livox_launch)
    except PackageNotFoundError:
        # Add warning if package is not found
        ld.add_action(LogInfo(msg="Warning: livox_ros_driver2 package not found. Skipping livox launch."))
    
    # Create ros2publisher node
    ros2publisher_node = Node(
        package='laser',
        executable='ros2publisher',
        name='modbus_sensor_node',
        # output='screen'
    )
    ld.add_action(ros2publisher_node)
    
    # Create myRreal node
    myRreal_node = Node(
        package='laser',
        executable='Real',
        name='myRreal_node',
        output='screen'
    )
    ld.add_action(myRreal_node)
    
    return ld
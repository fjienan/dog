from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Include livox_ros_driver2 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('livox_ros_driver2'),
                    'launch_ROS2',
                    'msg_MID360_launch.py'
                ])
            ]),
        ),
        
        # Include fast_lio mapping launch file with parameter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ]),
            launch_arguments={
                'config_file': 'avia.yaml'
            }.items()
        ),
        
        # Launch usb_bulk_node
        Node(
            package='usb_bulk_node',
            executable='usb_bulk_node',
            name='usb_bulk_node'
        ),
        
        # Launch udp_receive_node
        Node(
            package='udp_receive_node',
            executable='udp_receive_node',
            name='udp_receive_node'
        ),
        
        # Launch cmd.py script
        ExecuteProcess(
            cmd=['python3', '~/scripts/cmd.py'],
            name='cmd_script',
            shell=True
        )
    ])
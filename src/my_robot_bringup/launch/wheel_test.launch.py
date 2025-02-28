from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取绝对路径
    pkg_dir = FindPackageShare('my_robot_description').find('my_robot_description')
    world_file = os.path.join(pkg_dir, 'worlds/robocon25.world')
     
    return LaunchDescription([
        # 1. 核心修复：必须首先启动Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch/gazebo.launch.py'
                ])
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # 2. 机器人模型生成（需在gazebo启动后）
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot',
                '-z', '0.1'
            ],
            delay=5.0  # 确保gazebo完全启动
        ),

        # 3. 状态发布节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(
                    os.path.join(pkg_dir, 'urdf/my_robot.urdf.xacro'),
                    'r'
                ).read()
            }]
        ),

        # 4. ROS2控制节点（关键基础设施）
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                os.path.join(pkg_dir, 'config/wheel_controllers.yaml'),
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        # 5. 控制器激活（必须在ros2_control_node启动后）
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=LaunchDescription.get_launch_description().entities[3],  # 索引对应ros2_control_node
                on_start=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['wheel_control'],
                        output='screen'
                    )
                ]
            )
        )
    ])

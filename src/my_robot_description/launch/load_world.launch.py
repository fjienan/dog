import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robocon25_sim = get_package_share_directory('my_robot_description')
    world_path = os.path.join(pkg_robocon25_sim, 'worlds', 'robocon25.world')
    urdf_path = os.path.join(pkg_robocon25_sim, 'urdf', 'my_robot.urdf')

    # 读取 URDF 文件内容
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 启动 Gazebo 服务
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'verbose': 'true',
            'world': world_path
        }.items()
    )

    # 发布机器人状态
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # 发布关节状态
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # 在 Gazebo 中生成机器人模型（确保在 Gazebo 启动后执行）
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',  # 从话题获取 URDF 内容
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # 确保在 Gazebo 完全启动后再生成机器人
    delayed_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_launch,
            on_exit=[spawn_entity],
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        delayed_spawn_entity
    ])

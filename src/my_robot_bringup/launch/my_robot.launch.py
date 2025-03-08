from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 修改 launch 文件中的 extra_gazebo_args 参数
extra_gazebo_args = '-s /opt/ros/humble/lib/libgazebo_ros_init.so -s /opt/ros/humble/lib/libgazebo_ros_factory.so'

def generate_launch_description():
    # ================== 多机器人配置 ================== 
    robot_configs = [
        # 己方车辆
        {
            "name": "robot1",
            "x": "-1.0",
            "y": "-1.0",
            "z": "0.1",
            "controller_config": "ally_controller.yaml",  # 己方通用控制器
            "team": "ally",
            "color": "Blue"  # 己方颜色
        },
        {
            "name": "robot2",
            "x": "111.0",
            "y": "2.0",
            "z": "0.1",
            "controller_config": "ally_controller.yaml",
            "team": "ally",
            "color": "Blue"
        },
        # 敌方车辆
        {
            "name": "robot3",
            "x": "111.0",
            "y": "0.0",
            "z": "0.1",
            "controller_config": "enemy_controller.yaml",  # 敌方专用控制器
            "team": "enemy",
            "color": "Red"  # 敌方颜色
        },
        {
            "name": "robot4",
            "x": "511.0",
            "y": "2.0",
            "z": "0.1",
            "controller_config": "enemy_controller.yaml",
            "team": "enemy",
            "color": "Red"
        }
    ]

    # ================== 共享配置 ================== 
    pkg_my_robot = FindPackageShare('my_robot_description')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    world_path = PathJoinSubstitution([pkg_my_robot, 'worlds', 'robocon25.world'])  # 战场环境
    
    launch_description = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={
                'world': world_path,
                'verbose': 'true',
                'extra_gazebo_args': '-s libgazebo_ros_init.so'  # 确保插件加载
                # 'extra_gazebo_args': '-s /opt/ros/humble/lib/libgazebo_ros_init.so -s /opt/ros/humble/lib/libgazebo_ros_factory.so'  # 绝对路径
            }.items()
        )
    ])

    # ================== 生成所有车辆 ================== 
    for config in robot_configs:
        robot_name = config["name"]
        team_ns = f"{config['team']}/{robot_name}"  # 命名空间结构: ally/robot1
        
        # URDF生成命令（添加队伍和颜色参数）
        xacro_command = Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg_my_robot, 'urdf', 'my_robot.urdf.xacro']),
            ' name:=', robot_name,
            ' team:=', config["team"],
            ' color:=', config["color"],
            ' init_x:=', config["x"],
            ' init_y:=', config["y"],
            ' init_z:=', config["z"]
        ])
        
        # 车辆专属配置
        vehicle_group = GroupAction([
            PushRosNamespace(team_ns),  # 双重命名空间 ally/robot1
            
            # 状态发布
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': xacro_command}],
                name=f'{robot_name}_state_pub'
            ),
            
            # Gazebo生成
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot_name,
                    '-robot_namespace', team_ns,
                    '-x', config["x"],
                    '-y', config["y"],
                    '-z', config["z"],
                    '-topic', 'robot_description'
                ],
                name=f'spawn_{robot_name}'
            ),
            
            # 控制器（根据队伍加载不同配置）
            # Node(
            #     package='controller_manager',
            #     executable='ros2_control_node',
            #     parameters=[
            #         PathJoinSubstitution([
            #             FindPackageShare('my_robot_bringup'),
            #             'config',
            #             config["controller_config"]
            #         ])
            #     ],
            #     name=f'{robot_name}_controller'
            # )
        ])
        
        launch_description.add_action(vehicle_group)

    # ================== 全局工具 ================== 
    launch_description.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('my_robot_bringup'), 'rviz', 'battlefield.rviz']
            )],
            name='battlefield_rviz'
        )
    )
    
    launch_description.add_action(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='arm_controller_gui'
        )
    )

    return launch_description
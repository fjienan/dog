from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 控制器管理器
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "wheel1_velocity_controller",
                "--controller-manager",
                "/controller_manager"
            ],
        )
    ])

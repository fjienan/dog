from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM1',
        description='串口设备路径'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='串口波特率'
    )

    timeout_arg = DeclareLaunchArgument(
        'read_timeout_ms',
        default_value='100',
        description='读取超时（毫秒）'
    )
    
    # 添加cmd_vel相关参数
    vel_func_id_arg = DeclareLaunchArgument(
        'vel_func_id',
        default_value='1',  # 功能ID
        description='速度控制的功能ID'
    )
    
    # linear_scale_arg = DeclareLaunchArgument(
    #     'linear_scale',
    #     default_value='1000.0',  # 缩放因子
    #     description='线速度缩放因子'
    # )
    
    # angular_scale_arg = DeclareLaunchArgument(
    #     'angular_scale',
    #     default_value='1000.0',  # 缩放因子 
    #     description='角速度缩放因子'
    # )

    # 创建节点
    ares_comm_node = Node(
        package='ares_comm',
        executable='ares_comm_node',
        name='ares_comm',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'read_timeout_ms': LaunchConfiguration('read_timeout_ms'),
            'vel_func_id': LaunchConfiguration('vel_func_id'),
            # 'linear_scale': LaunchConfiguration('linear_scale'),
            # 'angular_scale': LaunchConfiguration('angular_scale')
            }],
        output='screen'
    )

    # 返回启动描述
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        timeout_arg,
        vel_func_id_arg,
        # linear_scale_arg,
        # angular_scale_arg,
        ares_comm_node
    ]) 
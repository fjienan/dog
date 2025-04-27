from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='串口波特率'
    )
    
    # 创建IMU传感器节点
    imu_node = Node(
        package='ares_comm',
        executable='imu_sensor_node',
        name='imu_sensor',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'read_timeout_ms': 100,
            'quaternion_data_id': 0x0001,
            'acceleration_data_id': 0x0002,
        }]
    )
    
    # 返回启动描述
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        imu_node
    ]) 
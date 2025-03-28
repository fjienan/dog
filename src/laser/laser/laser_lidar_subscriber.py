#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import numpy as np

class MultiSensorSubscriber(Node):
    def __init__(self):
        super().__init__('multi_sensor_subscriber')
        
        # 订阅Modbus传感器数据
        self.modbus_sub = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.modbus_callback,
            10)
        
        # 订阅Livox IMU数据
        self.imu_sub = self.create_subscription(
            Imu,
            'livox/imu',
            self.imu_callback,
            10)
        
        self.get_logger().info("多传感器订阅者已启动，等待数据...")

    def modbus_callback(self, msg):
        """处理Modbus传感器数据"""
        try:
            sensor_values = []
            for i, value in enumerate(msg.data):
                if np.isnan(value):
                    sensor_values.append(f"传感器{i+1}: 无效数据")
                else:
                    sensor_values.append(f"传感器{i+1}: {value:.4f}m")
            
            self.get_logger().info(
                f"Modbus数据: {' | '.join(sensor_values)}",
                throttle_duration_sec=1.0  # 限制日志频率
            )
            
        except Exception as e:
            self.get_logger().error(f"处理Modbus数据出错: {str(e)}")

    def imu_callback(self, msg):
        """处理IMU数据"""
        try:
            # 提取四元数
            q = msg.orientation
            # 提取角速度 (rad/s)
            angular_vel = msg.angular_velocity
            # 提取线加速度 (m/s²)
            linear_acc = msg.linear_acceleration
            
            self.get_logger().info(
                f"IMU数据 - 姿态: [{q.w:.3f}, {q.x:.3f}, {q.y:.3f}, {q.z:.3f}] | "
                f"角速度: [{angular_vel.x:.3f}, {angular_vel.y:.3f}, {angular_vel.z:.3f}] rad/s | "
                f"加速度: [{linear_acc.x:.3f}, {linear_acc.y:.3f}, {linear_acc.z:.3f}] m/s²",
                throttle_duration_sec=0.5  # 限制高频IMU数据的日志频率
            )
            
            # 这里可以添加更多的IMU数据处理逻辑
            
        except Exception as e:
            self.get_logger().error(f"处理IMU数据出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    subscriber = MultiSensorSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info("订阅者节点终止")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
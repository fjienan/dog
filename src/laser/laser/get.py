import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/enemy/robot4/scan',
            self.scan_callback,
            10)
        self.subscription  # 防止未使用的变量警告
        
    def scan_callback(self, msg):
        # 获取激光雷达数据并且进行可视化，使用matplotlib
        self.get_logger().info('Laser scan received')
        # 计算距离最近的障碍物距离
        ranges = np.array(msg.ranges)
        self.real_laser = [ranges[0], ranges[72], ranges[144], ranges[216], ranges[288]]
        self.get_logger().info(f"Processed laser ranges: {self.real_laser}")

def main(args=None):
    rclpy.init(args=args)
    laser_subscriber = LaserSubscriber()
    
    rclpy.spin(laser_subscriber)
    
    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
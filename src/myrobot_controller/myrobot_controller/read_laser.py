import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # 创建订阅者，订阅激光雷达数据
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # 创建发布者，发布控制指令
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        self.subscription1 = self.create_subscription(
            LaserScan,
            'scan_fivelaser1',
            self.scan_callback1,
            10)
        self.subscription2 = self.create_subscription(
            LaserScan,
            'scan_fivelaser2',
            self.scan_callback2,
            10)
        self.subscription3 = self.create_subscription(
            LaserScan,
            'scan_fivelaser3',
            self.scan_callback3,
            10)
        self.subscription4 = self.create_subscription(
            LaserScan,
            'scan_fivelaser4',
            self.scan_callback4,
            10)
        self.subscription5 = self.create_subscription(
            LaserScan,
            'scan_fivelaser5',
            self.scan_callback5,
            10)
        # 初始化控制指令
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.five_scan=np.zeros((5,)) 
        # 设置障碍物检测的阈值
        self.obstacle_threshold = 1.0  # 1米
        while rclpy.ok():
            rclpy.spin_once(self)


    def scan_callback1(self, msg):
        self.five_scan[0] = min(msg.ranges)

    def scan_callback2(self, msg):
        self.five_scan[1] = min(msg.ranges)

    def scan_callback3(self, msg):
        self.five_scan[2] = min(msg.ranges)

    def scan_callback4(self, msg):
        self.five_scan[3] = min(msg.ranges)

    def scan_callback5(self, msg):
        self.five_scan[4] = min(msg.ranges)

    def scan_callback(self, msg):
        # 获取激光雷达数据并且进行可视化，使用pyqt
        self.get_logger().info('scan received')
        # 计算距离最近的障碍物距离
        
        
        
        # 发布控制指令
        self.publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    
    node = ObstacleAvoidanceNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

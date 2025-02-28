import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



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
        self.five_scan = np.zeros((5,)) 
        # 设置障碍物检测的阈值
        self.obstacle_threshold = 1.0  # 1米

        # 初始化matplotlib绘图
        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter([], [])
        self.five_scan_scatter = self.ax.scatter([], [], color='red')  # 用于显示 five_scan 的点
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.ax.set_aspect('equal')

        # 显示绘图窗口
        plt.ion()
        plt.show()

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
        # 获取激光雷达数据并且进行可视化，使用matplotlib
        self.get_logger().info('scan received')
        # 计算距离最近的障碍物距离
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # 更新绘图
        self.scatter.set_offsets(np.c_[x, y])

        # 计算 five_scan 的笛卡尔坐标
        five_angles = np.linspace(0, 2 * np.pi, 5, endpoint=False)
        five_x = self.five_scan * np.cos(five_angles)
        five_y = self.five_scan * np.sin(five_angles)


        # 更新 five_scan 的绘图
        self.five_scan_scatter.set_offsets(np.c_[five_x, five_y])
        self.five_scan=np.zeros((5,))
        # 标记图形画布需要重绘，但不会立即重绘，等待合适时机由系统自动处理
        self.fig.canvas.draw_idle()
        # 立即处理所有待处理的 GUI 事件，确保图形界面及时更新
        self.fig.canvas.flush_events()

        # 发布控制指令
        # 发布指令使其可以绕圆心旋转
        # self.cmd_vel.linear.x = 0.1
        # self.cmd_vel.angular.z = 0.5
        # self.publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    
    node = ObstacleAvoidanceNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

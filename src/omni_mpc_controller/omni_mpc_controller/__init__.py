from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class MultiRobotController(Node):
    def __init__(self):
        super().__init__('multi_robot_controller')

        # 订阅机器人1和机器人2的激光雷达数据
        self.create_subscription(LaserScan, '/ally/robot1/scan1', self.laser_callback_robot1, 10)
        self.create_subscription(LaserScan, '/ally/robot2/scan1', self.laser_callback_robot2, 10)
        
        # 订阅机器人1和机器人2的里程计数据
        self.create_subscription(Odometry, '/ally/robot1/odom', self.odom_callback_robot1, 10)
        self.create_subscription(Odometry, '/ally/robot2/odom', self.odom_callback_robot2, 10)
        
        # 订阅机器人1和机器人2的IMU数据
        self.create_subscription(Imu, '/ally/robot1/imu', self.imu_callback_robot1, 10)
        self.create_subscription(Imu, '/ally/robot2/imu', self.imu_callback_robot2, 10)

        # 订阅目标点话题
        # self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # 发布速度命令
        self.cmd_pub_robot1 = self.create_publisher(Twist, '/ally/robot1/cmd_vel', 10)
        self.cmd_pub_robot2 = self.create_publisher(Twist, '/ally/robot2/cmd_vel', 10)

    def laser_callback_robot1(self, msg):
        # 处理机器人1的激光雷达数据
        self.get_logger().info("接收到机器人1的激光雷达数据")
        
    def laser_callback_robot2(self, msg):
        # 处理机器人2的激光雷达数据
        self.get_logger().info("接收到机器人2的激光雷达数据")

    def odom_callback_robot1(self, msg):
        # 处理机器人1的里程计数据
        self.get_logger().info("接收到机器人1的里程计数据")

    def odom_callback_robot2(self, msg):
        # 处理机器人2的里程计数据
        self.get_logger().info("接收到机器人2的里程计数据")

    def imu_callback_robot1(self, msg):
        # 处理机器人1的IMU数据
        self.get_logger().info("接收到机器人1的IMU数据")

    def imu_callback_robot2(self, msg):
        # 处理机器人2的IMU数据
        self.get_logger().info("接收到机器人2的IMU数据")

    def goal_callback(self, msg):
        # 处理目标点数据
        self.get_logger().info("接收到目标点数据")

    def control_robot1(self, cmd: Twist):
        # 控制机器人1的运动
        self.cmd_pub_robot1.publish(cmd)

    def control_robot2(self, cmd: Twist):
        # 控制机器人2的运动
        self.cmd_pub_robot2.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

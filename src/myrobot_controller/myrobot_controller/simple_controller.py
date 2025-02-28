#!/usr/bin/env python3

# 导入必要的库
import rclpy  # ROS 2 Python 客户端库
from rclpy.node import Node  # Node 类用于创建 ROS 节点
from std_msgs.msg import Float64MultiArray  # 用于发布轮速命令的消息类型
from geometry_msgs.msg import TwistStamped  # 用于接收速度命令的消息类型
import numpy as np  # 用于数值计算的库

# 定义 SimpleController 类，继承自 Node
class SimpleController(Node):
    def __init__(self):
        # 初始化节点，节点名称为 'simple_controller'
        super().__init__('simple_controller')

        # 声明参数，轮子半径和轮子间距
        self.declare_parameter('wheel_radius', 0.033)  # 轮子半径，单位为米
        self.declare_parameter('wheel_separation', 0.17)  # 轮子间距，单位为米

        # 获取参数值
        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value

        # 记录使用的参数值
        self.get_logger().info('Using wheel_radius %f' % self.wheel_radius_)
        self.get_logger().info('Using wheel_separation %f' % self.wheel_separation_)

        # 创建发布者，用于发布轮速命令
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        # 创建订阅者，用于接收速度命令
        self.vel_sub_ = self.create_subscription(TwistStamped, "myrobot_controller/cmd_vel", self.velcallback, 10)

        # 计算速度转换矩阵
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation, -self.wheel_radius_/self.wheel_separation]])

        # 记录转换矩阵
        self.get_logger().info('The conversion matrix is %s' % self.speed_conversion_)

    # 速度回调函数
    def velcallback(self, msg):
        # 从接收到的消息中提取机器人速度
        robot_speed = np.array([msg.twist.linear.x, msg.twist.angular.z])
        # 计算轮速
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        # 创建轮速消息
        wheel_speed_msg = Float64MultiArray()
        # 设置轮速数据
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        # 发布轮速消息
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

# 主函数
def main():
    rclpy.init()  # 初始化 ROS 2
    simple_controller = SimpleController()  # 创建 SimpleController 实例
    rclpy.spin(simple_controller)  # 进入循环，等待消息
    simple_controller.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 ROS 2

# 程序入口
if __name__ == '__main__':
    main()
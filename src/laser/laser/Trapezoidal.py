#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
import math
import tf_transformations

class OmnidirectionalController(Node):
    def __init__(self):
        super().__init__('omnidirectional_controller')
        
        # 目标点 (x=2.0, y=0.0)
        self.current_yaw = None
        self.target_x = 0.6
        self.target_y = 0.0
        self.
        
        # 最大速度限制
        self.max_linear_speed = 2.0
        
        # 比例增益 (可调)
        self.Kp = 0.5  # 降低增益减少超调
        
        # 订阅 'base_link/world_position'（假设是 geometry_msgs/msg/Point）
        self.subscription = self.create_subscription(
            Point,
            'base_link/world_position',
            self.point_callback,
            10
        )
        
        self.subscription_angle = self.creat_subscription(
            Float64,
            'yaw_angle',
            self.assign_angular_callback,
            10
        )
        
        # 发布 cmd_vel（Twist 消息）
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.get_logger().info("Omnidirectional Controller started. Target: (1.0, 1.0)")
    
    def assign_angular_callback(self, msg):
        self.current_yaw = msg.data
    
        
        
    def point_callback(self, msg):
        """
        计算当前点到目标点的误差，并发布 cmd_vel
        """
        current_x = msg.x
        current_y = msg.y
        if self.current_yaw is not None:
            current_angle = self.current_yaw
            error_yaw = self.target_yaw - current_yaw
        # 计算误差（目标 - 当前）
        error_x = self.target_x - current_x
        error_y = self.target_y - current_y
        
        
        # 计算距离（用于判断是否到达）
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # 如果距离很小，停止运动
        if distance < 0.05:  # 阈值可调
            self.get_logger().info("Reached target!")
            cmd_vel = Twist()
        else:
            # 直接使用误差作为控制量（不归一化）
            cmd_vel = Twist()
            cmd_vel.linear.x = self.Kp * error_x
            cmd_vel.linear.y = self.Kp * error_y
            
            # 限速处理
            linear_speed = math.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2)
            if linear_speed > self.max_linear_speed:
                scale = self.max_linear_speed / linear_speed
                cmd_vel.linear.x *= scale
                cmd_vel.linear.y *= scale
        
        # 发布 cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(
            f"Current: ({current_x:.2f}, {current_y:.2f}) | "
            f"Cmd_vel: (x={cmd_vel.linear.x:.2f}, y={cmd_vel.linear.y:.2f})"
            f"distance: (x={distance:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OmnidirectionalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
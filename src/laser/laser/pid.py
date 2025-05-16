#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations
import numpy as np
from transformation import Transformation

"""
import reasonable values for the transformation
see the explaination in the transforamtion.py
"""
tranformation = Transformation(angle = 2*np.pi/3 ,turn_over = True ,dimension = 2)
class OmnidirectionalController(Node):
    def __init__(self):
        super().__init__('omnidirectional_controller')
        
        # 目标点 (x=2.0, y=0.0)
        self.current_yaw = None
        self.target_x = -0.0
        self.target_y = -5.53
        self.target_yaw = -1.57
        
        # 最大速度限制
        self.max_linear_speed = 2.0
        self.max_angular_speed = 1.0
        # 比例增益 (可调)
        self.Kp = 0.5  # 降低增益减少超调
        self.Kp_for_angular = 0.5
        
        # 订阅 'base_link/world_position'（假设是 geometry_msgs/msg/Point）
        self.subscription = self.create_subscription(
            PoseStamped,
            'laser_position',
            self.point_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'Odometry',
            self.assign_angular_callback,
            10
        )
        
        # 发布 cmd_vel（Twist 消息）
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Omnidirectional Controller started. Target: (1.0, 1.0)")
    
    def assign_angular_callback(self, msg):
        orientation = msg.pose.pose.orientation
        
    
        # 使用tf_transformations转换四元数到欧拉角
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        
        self.current_yaw = yaw
        
        
    def point_callback(self, msg):
        """
        计算当前点到目标点的误差，并发布 cmd_vel
        """
        if self.current_yaw is  None:
            self.current_yaw = self.target_yaw

        current_x = msg.x
        current_y = msg.y
        current_angle = self.current_yaw

        
        
        # 计算误差（目标 - 当前）
        error_x = self.target_x - current_x
        error_y = self.target_y - current_y
        error_yaw = self.target_yaw - self.current_yaw
        print(error_yaw,"here")
        
        # 计算距离（用于判断是否到达）
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # 如果距离很小，停止运动
        if distance < 0.05 and abs(error_yaw) <0.05:  # 阈值可调
            self.get_logger().info("Reached target!")
            cmd_vel = Twist()
        else:
            # 直接使用误差作为控制量（不归一化）
            cmd_vel = Twist()
            cmd_vel.linear.x = self.Kp * error_x
            cmd_vel.linear.y = self.Kp * error_y
            cmd_vel.angular.x = -self.Kp_for_angular * error_yaw
            
            # 限速处理
            linear_speed = math.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2)
            if linear_speed > self.max_linear_speed:
                scale = self.max_linear_speed / linear_speed
                cmd_vel.linear.x *= scale
                cmd_vel.linear.y *= scale
            if abs(cmd_vel.angular.x) > self.max_angular_speed:
                scale = self.max_angular_speed/cmd_vel.angular.x
                cmd_vel.angular.x *= scale
        
        # 发布 cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(
            f"Current: ({current_x:.2f}, {current_y:.2f},{current_angle:.2f}) | "
            f"Cmd_vel: (x={cmd_vel.linear.x:.2f}, y={cmd_vel.linear.y:.2f},z={cmd_vel.angular.x:.2f})"
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
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, TransformStamped  # 新增导入
import math
from tf2_ros import TransformBroadcaster
import numpy as np


class OdomAngleCalculator(Node):
    def __init__(self):
        super().__init__('odom_angle_calculator')
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            'Odometry',
            self.odom_callback,
            10)
        
        # 发布移动角度到新话题
        self.angle_pub = self.create_publisher(Float64, 'movement_angle', 10)

        # 新增: 发布当前偏航角到yaw_angle话题
        self.yaw_angle_pub = self.create_publisher(Float64, 'yaw_angle', 10)
        
        # 新增: 发布body坐标到point1话题
        self.point1_pub = self.create_publisher(Point, 'base_link/world_position', 10)
        
        # 新增: 发布laser坐标到point2话题
        self.point2_pub = self.create_publisher(Point, 'laser_link/world_position', 10)
       # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self) 
        # 存储数据
        self.latest_odom = None
        self.initial_position = None
        self.initial_angle = None
        self.last_position = None
        
        self.get_logger().info("Odom Angle Calculator 已启动")

    def laser_to_world(self, x, y, angle):
        convert_y = y*np.cos(angle) - x*np.sin(angle)
        convert_x = -y*np.sin(angle) - x*np.cos(angle)
        return convert_x, convert_y

    def odom_callback(self, msg):
        """里程计回调函数"""
        # 存储最新里程计数据
        self.latest_odom = msg
        current_position = msg.pose.pose.position
        angle = np.pi/6
        bias = 0.42
        
        # 计算当前航向角度
        current_angle = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # 坐标转换
        y = current_position.y
        x = current_position.x
        
        body_x_laser = x + bias*np.sin(current_angle)
        body_y_laser = y - bias*np.cos(current_angle)
        body_x_world, body_y_world = self.laser_to_world(body_x_laser, body_y_laser+0.42, angle)
        laser_x_world, laser_y_world = self.laser_to_world(x, y+0.42, angle)
       
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_init'  # parent frame
        t.child_frame_id = 'base_link'     # child frame 

        # 设置变换 (body_x_laser, body_y_laser, 0)
        t.transform.translation.x = float(body_x_laser)
        t.transform.translation.y = float(body_y_laser)
        t.transform.translation.z = float(current_position.z)
        
        t.transform.rotation = msg.pose.pose.orientation

         # 广播TF变换
        self.tf_broadcaster.sendTransform(t)
        # 发布当前偏航角到yaw_angle话题
        yaw_msg = Float64()
        yaw_msg.data = current_angle
        self.yaw_angle_pub.publish(yaw_msg)
        # 新增: 发布body坐标到point1话题
        point1_msg = Point()
        point1_msg.x = float(body_x_world)
        point1_msg.y = float(body_y_world)
        point1_msg.z = 0.0  # 2D坐标，z设为0
        self.point1_pub.publish(point1_msg)
        
        # 新增: 发布laser坐标到point2话题
        point2_msg = Point()
        point2_msg.x = float(laser_x_world)
        point2_msg.y = float(laser_y_world)
        point2_msg.z = 0.0  # 2D坐标，z设为0
        self.point2_pub.publish(point2_msg)


        
        # 记录初始位置
        if self.initial_position is None:
            self.initial_position = current_position
            self.initial_angle = self.quaternion_to_yaw(msg.pose.pose.orientation)
            self.get_logger().info(
                f"初始位置记录: x={self.initial_position.x:.2f}, "
                f"y={self.initial_position.y:.2f}, "
                f"角度={math.degrees(self.initial_angle):.1f}°")
        
        # 总是更新最后位置
        self.last_position = current_position
        
        
        
        # 计算相对于初始位置的移动角度
        if self.initial_position is not None:
            dx = current_position.x - self.initial_position.x
            dy = current_position.y - self.initial_position.y
            movement_angle = math.atan2(dy, dx)
            
            # 发布移动角度
            angle_msg = Float64()
            angle_msg.data = movement_angle
            self.angle_pub.publish(angle_msg)
            
            # 计算角度差
            angle_diff = self.normalize_angle(current_angle - movement_angle)
            
            self.get_logger().info(
                f"\n当前位置: x={current_position.x:.2f}, y={current_position.y:.2f}\n"
                f"body_laser坐标: x={body_x_laser:.2f}, y={body_y_laser:.2f}\n"
                f"body坐标: x={body_x_world:.2f}, y={body_y_world:.2f}\n"
                f"laser坐标: x={laser_x_world:.2f}, y={laser_y_world:.2f}\n"
                f"当前角度: {math.degrees(current_angle):.1f}°, "
                
                ,throttle_duration_sec=0.2)  # 限制日志频率
    
    def quaternion_to_yaw(self, quat):
        """将四元数转换为偏航角（弧度）"""
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        
        # 计算偏航角
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围内"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def get_latest_odom(self):
        """获取最新的里程计消息"""
        return self.latest_odom
    
    def get_current_angle(self):
        """获取当前航向角度（弧度）"""
        if self.latest_odom is None:
            return None
        return self.quaternion_to_yaw(self.latest_odom.pose.pose.orientation)

def main(args=None):
    rclpy.init(args=args)
    node = OdomAngleCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 节点停止时打印终止位置
        if node.last_position is not None:
            node.get_logger().info(
                f"终止位置: x={node.last_position.x:.2f}, y={node.last_position.y:.2f}")
        node.get_logger().info("接收到Ctrl+C，停止节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
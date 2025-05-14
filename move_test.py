#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class FixedCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('fixed_cmd_vel_publisher_py')
        
        # --- ROS 2 初始化 ---
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建定时器，每10毫秒执行一次
        timer_period = 0.01  # 10毫秒 = 0.01秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 初始化计数器
        self.info_cnt = 0
        
        self.get_logger().info('Fixed Cmd Vel Publisher n`ode (Python) started.')
        self.get_logger().info('Publishing fixed velocity: linear.x = 0.1 m/s')
    
    def timer_callback(self):
        # 创建一个Twist消息
        msg = Twist()
        
        # 设置固定的线速度（x方向0.1m/s）
        msg.linear.x = 0.0  # x方向速度为0.1 m/s
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # 设置角速度为0
        msg.angular.x = 1.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 每100次发布打印一次日志
        self.info_cnt += 1
        if self.info_cnt % 100 == 0:
            self.get_logger().info(
                f'Publishing cmd: Linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}), '
                f'Angular=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        fixed_cmd_vel_publisher = FixedCmdVelPublisher()
        rclpy.spin(fixed_cmd_vel_publisher)
    except Exception as e:
        print(f'Error during node initialization or runtime: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        # 确保 ROS 关闭
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
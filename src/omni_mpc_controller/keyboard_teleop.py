#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time
import numpy as np

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/manual_cmd', 10)
        
        # 参数
        self.declare_parameters(namespace='',
            parameters=[
                ('max_speed', 2.0),           # 最大速度
                ('acceleration', 0.5),        # 降低加速度，使控制更平滑
                ('friction', 0.2),            # 摩擦系数
                ('control_rate', 20.0)        # 控制率
            ])
        
        # 获取参数
        self.max_speed = self.get_parameter('max_speed').value
        self.acceleration = self.get_parameter('acceleration').value
        self.friction = self.get_parameter('friction').value
        control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / control_rate
        
        # 当前速度和控制状态
        self.vx = 0.0
        self.vy = 0.0
        self.active_keys = set()  # 当前活动的按键集合
        self.control_vector = np.array([0.0, 0.0])  # 控制方向向量
        self.last_key_press_time = time.time()  # 记录最后一次按键时间
        self.key_timeout = 0.5  # 如果超过这个时间没有按键，认为按键已释放
        
        # 创建控制定时器
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # 启动键盘监听线程
        self.stop_requested = False
        self.keyboard_thread = threading.Thread(target=self.read_keys)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('键盘遥控已启动（更平滑的加速度控制）')
        self.print_usage()
    
    def print_usage(self):
        self.get_logger().info("""
键盘控制说明:
---------------------------
按键控制:
    w - 向前加速     s - 向后加速
    a - 向左加速     d - 向右加速
    q - 左前加速     e - 右前加速
    z - 左后加速     c - 右后加速
    
方向键控制:
    ↑ - 向前加速     ↓ - 向后加速
    ← - 向左加速     → - 向右加速
    
其他:
    空格 - 急停
    Ctrl+C - 退出
""")
    
    def timer_callback(self):
        """定时更新和发布速度"""
        # 检查按键超时
        if time.time() - self.last_key_press_time > self.key_timeout:
            self.active_keys.clear()
            self.control_vector = np.array([0.0, 0.0])
        
        # 根据控制向量和当前按键状态更新速度
        if np.linalg.norm(self.control_vector) > 0.01:
            # 按键激活时，按控制向量方向加速，加速度较小
            accel = self.control_vector * self.acceleration * self.dt
            self.vx += accel[0]
            self.vy += accel[1]
            self.get_logger().info(f'按照方向 [{self.control_vector[0]:.1f}, {self.control_vector[1]:.1f}] 加速')
        else:
            # 无按键时，应用摩擦力减速
            speed = np.sqrt(self.vx**2 + self.vy**2)
            if speed > 0.01:
                # 计算减速量
                decel = min(self.friction * self.dt, speed)
                # 按当前速度方向减速
                decel_factor = max(0, (speed - decel) / speed)
                self.vx *= decel_factor
                self.vy *= decel_factor
                if decel > 0:
                    self.get_logger().debug(f'自然减速: 减速因子 {decel_factor:.2f}')
            else:
                # 速度很小时直接归零
                self.vx = 0.0
                self.vy = 0.0
        
        # 速度限制
        speed = np.sqrt(self.vx**2 + self.vy**2)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            self.vx *= scale
            self.vy *= scale
        
        # 创建并发布消息
        msg = Twist()
        msg.linear.x = float(self.vx)
        msg.linear.y = float(self.vy)
        msg.angular.z = 0.0
        
        self.publisher.publish(msg)
        
        # 只在速度变化明显时输出日志
        if abs(self.vx) > 0.05 or abs(self.vy) > 0.05:
            self.get_logger().info(f'当前速度: [{self.vx:.2f}, {self.vy:.2f}] m/s')
    
    def update_control_vector(self):
        """根据当前活动按键更新控制向量"""
        # 默认控制向量为零
        vec = np.zeros(2)
        
        # 根据按下的按键设置控制向量分量
        if 'w' in self.active_keys or 'up' in self.active_keys: vec += [0, 1]    # 前
        if 's' in self.active_keys or 'down' in self.active_keys: vec += [0, -1]   # 后
        if 'a' in self.active_keys or 'left' in self.active_keys: vec += [-1, 0]   # 左
        if 'd' in self.active_keys or 'right' in self.active_keys: vec += [1, 0]    # 右
        if 'q' in self.active_keys: vec += [-0.7, 0.7]  # 左前
        if 'e' in self.active_keys: vec += [0.7, 0.7]   # 右前
        if 'z' in self.active_keys: vec += [-0.7, -0.7] # 左后
        if 'c' in self.active_keys: vec += [0.7, -0.7]  # 右后
        
        # 空格键特殊处理 - 急停
        if ' ' in self.active_keys:
            self.vx = 0.0
            self.vy = 0.0
            vec = np.zeros(2)
            self.get_logger().info('急停!')
        
        # 归一化控制向量
        norm = np.linalg.norm(vec)
        if norm > 0:
            self.control_vector = vec / norm
        else:
            self.control_vector = vec
    
    def read_keys(self):
        """读取键盘输入"""
        while not self.stop_requested:
            # 获取终端设置
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                # 设置终端为raw模式
                tty.setraw(fd)
                # 读取单个字符
                ch = sys.stdin.read(1)
                
                # 检测方向键序列
                if ch == '\x1b':  # ESC
                    ch1 = sys.stdin.read(1)
                    if ch1 == '[':
                        ch2 = sys.stdin.read(1)
                        if ch2 == 'A': # 上
                            self.process_key('up')
                        elif ch2 == 'B': # 下
                            self.process_key('down')
                        elif ch2 == 'C': # 右
                            self.process_key('right')
                        elif ch2 == 'D': # 左
                            self.process_key('left')
                else:
                    self.process_key(ch)
                    
            finally:
                # 恢复终端设置
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
            if ch == '\x03':  # Ctrl+C
                self.stop_requested = True
                self.get_logger().info("正在关闭键盘控制...")
                break
            
            time.sleep(0.01)  # 小延迟防止CPU占用过高
    
    def process_key(self, key):
        """处理按键，每次按键增加一点速度"""
        valid_keys = ['w', 'a', 's', 'd', 'q', 'e', 'z', 'c', ' ',
                      'up', 'down', 'left', 'right']
        if key in valid_keys:
            self.active_keys.add(key)
            self.last_key_press_time = time.time()
            self.update_control_vector()
            
            # 每次按键只应用一小段时间的加速度
            if key != ' ':  # 空格是急停，不需要下面的处理
                self.get_logger().info(f'按键 {key} 加速')
                # 5次定时器周期后清除按键状态
                self.active_keys.clear()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("键盘中断，正在关闭...")
    finally:
        # 确保停止机器人
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.stop_requested = True
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
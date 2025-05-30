#!/usr/bin/env python3
# 键盘坐标发布节点 - 持续发布最后输入的二维坐标
# 功能：输入新坐标前，节点以10Hz频率发布前一组坐标数据
# 消息类型：geometry_msgs/msg/Point (x, y, z)，其中z固定为0
# 引用：[4,6,7](@ref)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import threading

class KeyboardCoordinatePublisher(Node):
    def __init__(self):
        super().__init__('keyboard_coordinate_publisher')
        
        self.declare_parameters(
            namespace = '',
            parameters = [
                ('dt', 0.0),
                ('world_size',[0.0,0.0])
            ]
        )
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.world_size = self.get_parameter('world_size').get_parameter_value().double_array_value

        self.get_logger().info(f"""
        dt: {self.dt}
        world_size: {self.world_size}
        """)
        
        # 创建发布者，话题名为 /input_coordinates，消息类型为Point
        self.publisher_ = self.create_publisher(Point, '/input_coordinates', 10)
        self.get_logger().info("节点已启动！等待键盘输入坐标（格式：x y）...")
        
        # 存储当前坐标的变量，初始为None
        self.current_coordinate = None
        
        # 创建定时器（10Hz）用于持续发布坐标
        self.timer = self.create_timer(self.dt, self.timer_callback)  # 0.1秒 = 10Hz
        
        # 启动键盘监听线程（非阻塞）
        self.keyboard_thread = threading.Thread(target=self.listen_keyboard)
        self.keyboard_thread.daemon = True  # 设为守护线程，随主线程退出
        self.keyboard_thread.start()

    def timer_callback(self):
        """定时器回调：发布当前存储的坐标"""
        if self.current_coordinate is not None:
            msg = Point()
            msg.x = self.current_coordinate[0]
            msg.y = self.current_coordinate[1]
            msg.z = 0.0  # 二维坐标，z固定为0
            self.publisher_.publish(msg)
            # self.get_logger().info(f"发布坐标: ({msg.x:.2f}, {msg.y:.2f})")

    def listen_keyboard(self):
        """独立线程：监听键盘输入并更新坐标"""
        while rclpy.ok():
            try:
                # 等待用户输入（格式：x y）
                user_input = input("请输入坐标 (x y) > ")
                parts = user_input.split()
                
                if len(parts) == 2 :
                    if 0.0 <= float(parts[0]) <= self.world_size[0] and 0.0 <= float(parts[1]) <= self.world_size[1]:
                        x = float(parts[0])
                        y = float(parts[1])
                        self.current_coordinate = (x, y)
                        self.get_logger().info(f"新坐标已更新: ({x}, {y})")
                    else:
                        self.get_logger().error("输入的坐标超出了世界范围")
                else:
                    self.get_logger().error("输入格式错误！请使用 'x y' 格式（例如：3.5 2.0）")
            except ValueError:
                self.get_logger().error("输入值无效！请确保输入数字")
            except Exception as e:
                self.get_logger().error(f"输入异常: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCoordinatePublisher()
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
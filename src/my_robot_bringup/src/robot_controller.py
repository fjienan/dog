import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher1 = self.create_publisher(Float64, 'my_robot/base_wheel1_controller/effort', 10)
        self.publisher2 = self.create_publisher(Float64, 'my_robot/base_wheel2_controller/effort', 10)
        self.publisher3 = self.create_publisher(Float64, 'my_robot/base_wheel3_controller/effort', 10)

    def send_commands(self):
        msg1 = Float64()
        msg1.data = 0.5  # 设置 wheel1 的力矩
        self.publisher1.publish(msg1)

        msg2 = Float64()
        msg2.data = 0.5  # 设置 wheel2 的力矩
        self.publisher2.publish(msg2)

        msg3 = Float64()
        msg3.data = 0.5  # 设置 wheel3 的力矩
        self.publisher3.publish(msg3)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    while rclpy.ok():
        controller.send_commands()
        rclpy.spin_once(controller, timeout_sec=0.1)

if __name__ == '__main__':
    main()

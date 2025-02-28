import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

class ImuYawSubscriber(Node):
    def __init__(self):
        super().__init__('imu_yaw_subscriber')
        # 创建订阅者，订阅 "imu" 话题，消息类型为 Imu，回调函数为 self.listener_callback
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用的变量警告

    def listener_callback(self, msg):
        # 从消息中获取四元数
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # 将四元数转换为欧拉角
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # 打印 yaw 角
        self.get_logger().info(f"Yaw angle from IMU: {yaw}")

def main(args=None):
    rclpy.init(args=args)
    # 创建 ImuYawSubscriber 节点
    imu_yaw_subscriber = ImuYawSubscriber()
    # 进入节点的循环
    rclpy.spin(imu_yaw_subscriber)
    # 销毁节点
    imu_yaw_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
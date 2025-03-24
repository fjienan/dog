from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class Soliloquist(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 声明所有参数并设置默认值
        self.declare_parameters(
            namespace='',
            parameters=[
                ('DT', 0.1),
                ('START_POINT', [7.5, 4.0]),
                ('LASER_ALLOWED_NOISE', 2.0),
                ('process_noise_std', [0.1, 0.1, 0.05]),
                ('measurement_noise_std', [0.5, 0.5, 0.1]),
                ('FIELD_SIZE', [15, 8]),
                ('FREQUENCY', 0.2)
            ]
        )
        
        # 获取参数值
        # self.time_cycle = self.get_parameter('time_cycle_s').value
        # self.output_str = self.get_parameter('output_str').value
        self.DT = self.get_parameter('DT').value
        self.START_POINT = self.get_parameter('START_POINT').value
        
        # 打印参数（调试用）
        self.get_logger().info(f"Loaded DT: {self.DT}")
        self.get_logger().info(f"Loaded FIELD_SIZE: {self.get_parameter('FIELD_SIZE').value}")
        
        # 启用参数回调
        # self.add_on_set_parameters_callback(self.param_callback)
        # self.timer = self.create_timer(self.time_cycle, self.timer_callback)
    
    def timer_callback(self):
        pass
        # self.get_logger().info(self.output_str)
    
    # def param_callback(self, params):
    #     # 动态更新参数
    #     for param in params:
    #         if param.name == 'output_str':
    #             self.output_str = param.value
    #         elif param.name == 'time_cycle_s':
    #             self.time_cycle = param.value
    #             self.timer.reset()  # 重新设置定时器周期
    #     return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = Soliloquist('laser_position')  # 节点名称必须与 YAML 中的命名空间一致
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
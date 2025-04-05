#!/usr/bin/env python3
import minimalmodbus
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# 设备配置
DEVICE_MAP = [
    {'port': '/dev/ttyCH9344USB3', 'address': 2},
    {'port': '/dev/ttyCH9344USB4', 'address': 1},
    {'port': '/dev/ttyCH9344USB0', 'address': 5},
    {'port': '/dev/ttyCH9344USB2', 'address': 3},
    {'port': '/dev/ttyCH9344USB1', 'address': 4}
]

# 全局参数
BAUDRATE = 115200
TIMEOUT = 0.5
REGISTER_ADDR = 0x15

class ModbusSensorNode(Node):
    def __init__(self):
        super().__init__('modbus_sensor_node')
        
        # 初始化仪器并保存端口信息
        self.instruments = []
        for device in DEVICE_MAP:
            inst = self.init_instrument(device['port'], device['address'])
            if inst is not None:
                inst.port_name = device['port']  # 添加自定义属性保存端口名
                self.instruments.append(inst)
        
        # 创建发布者
        self.publisher = self.create_publisher(Float32MultiArray, 'sensor_data', 10)
        
        # 创建定时器 (100ms周期)
        self.timer = self.create_timer(0.04, self.read_and_publish)
        
        self.get_logger().info("Modbus传感器节点已启动")

    def init_instrument(self, port, address):
        """初始化仪器实例"""
        try:
            inst = minimalmodbus.Instrument(port, address)
            inst.serial.baudrate = BAUDRATE
            inst.serial.bytesize = 8
            inst.serial.parity = 'N'
            inst.serial.stopbits = 1
            inst.serial.timeout = TIMEOUT
            inst.clear_buffers_before_each_transaction = True
            inst.mode = minimalmodbus.MODE_RTU
            return inst
        except Exception as e:
            self.get_logger().error(f"初始化端口{port}失败: {str(e)}")
            return None

    def read_sensor(self, inst):
        """带重试机制的传感器读取"""
        for retry in range(3):
            try:
                registers = inst.read_registers(REGISTER_ADDR, 2)
                return (registers[0] << 16) + registers[1]
            except minimalmodbus.NoResponseError:
                self.get_logger().warn(
                    f"{inst.port_name}地址{inst.address}无响应，第{retry+1}次重试...",
                    throttle_duration_sec=5
                )
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"{inst.port_name}地址{inst.address}通信错误: {str(e)}")
                break
        return None

    def read_and_publish(self):
        """依次读取并发布所有传感器数据"""
        try:
            distances = []
            start_time = time.perf_counter()
            
            # 依次读取每个传感器
            for inst in self.instruments:
                result = self.read_sensor(inst)
                distance = result/10000.0 if result is not None else float('nan')
                distances.append(distance)
            
            # 创建并发布消息
            msg = Float32MultiArray()
            msg.data = distances
            self.publisher.publish(msg)
            # self.get_logger().info(
            #     f"发布数据: {[f'{d:.4f}' if not d!=d else 'NaN' for d in distances]}",
            #     throttle_duration_sec=1
            # )
            # 计算并记录耗时
            # total_time = (time.perf_counter() - start_time) * 1000
            # self.get_logger().info(
            #     f"数据: {[f'{d:.4f}' if not d!=d else 'NaN' for d in distances]} | 耗时: {total_time:.2f}ms",
            #     throttle_duration_sec=1
            # )
            
        except Exception as e:
            self.get_logger().error(f"系统错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ModbusSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
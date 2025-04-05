#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "ares_comm/serial_interface.hpp"
#include "ares_comm/protocol_parser.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ares_comm {

struct QuaternionData {
    float x;
    float y;
    float z;
    float w;
};

struct AccelerationData {
    float x;
    float y;
    float z;
};

class IMUSensorNode : public rclcpp::Node {
public:
    IMUSensorNode() : Node("imu_sensor_node") {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("read_timeout_ms", 100);
        this->declare_parameter<int>("quaternion_data_id", 0x0001);
        this->declare_parameter<int>("acceleration_data_id", 0x0002);

        // 获取参数
        std::string port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        read_timeout_ms_ = this->get_parameter("read_timeout_ms").as_int();
        quaternion_data_id_ = this->get_parameter("quaternion_data_id").as_int();
        acceleration_data_id_ = this->get_parameter("acceleration_data_id").as_int();

        // 创建四元数发布者
        quaternion_publisher_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
            "imu/quaternion", 10);
            
        // 创建加速度发布者
        acceleration_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "imu/acceleration", 10);

        // 打开串口
        try {
            serial_ = SerialInterface::create(port, baudrate);
            RCLCPP_INFO(this->get_logger(), "成功打开串口 %s，波特率 %d", 
                        port.c_str(), baudrate);
        } catch (const SerialException& e) {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败: %s", e.what());
            return;
        }

        // 创建定时器，定期从串口读取数据
        timer_ = this->create_wall_timer(
            10ms, std::bind(&IMUSensorNode::read_serial, this));
    }
    
    ~IMUSensorNode() {
        if (serial_ && serial_->is_open()) {
            serial_->close();
        }
    }

private:
    void read_serial() {
        if (!serial_ || !serial_->is_open()) {
            return;
        }

        try {
            // 尝试读取一个完整的帧
            std::vector<uint8_t> frame = serial_->read_frame();
            if (frame.empty()) {
                return;
            }

            // 打印接收到的原始帧（十六进制格式）
            std::stringstream hex_data;
            hex_data << "接收到数据帧: ";
            for (uint8_t byte : frame) {
                hex_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            // RCLCPP_INFO(this->get_logger(), "%s", hex_data.str().c_str());

            // 检查是否是同步帧(0x5A5A)
            if (frame.size() >= 2 && frame[0] == 0x5A && frame[1] == 0x5A) {
                // 检查帧尾(0xAA55)
                size_t frame_len = frame.size();
                if (frame_len >= 7 && frame[frame_len-2] == 0xAA && frame[frame_len-1] == 0x55) {
                    // 提取数据ID
                    uint16_t data_id = (frame[2] << 8) | frame[3];
                    RCLCPP_DEBUG(this->get_logger(), "接收到同步帧数据ID: 0x%04X", data_id);
                    
                    // 处理四元数数据
                    if (data_id == quaternion_data_id_) {
                        // 检查帧长度至少包含头部(2字节)+数据ID(2字节)+四元数(16字节)+校验(1字节)+尾部(2字节)
                        if (frame_len >= 4 + sizeof(QuaternionData) + 3) {
                            QuaternionData quat_data;
                            // 使用memcpy从数据帧提取四元数
                            memcpy(&quat_data, &frame[4], sizeof(QuaternionData));
                            
                            // 打印四元数数据
                            // RCLCPP_INFO(this->get_logger(), 
                            //             "四元数数据: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
                            //             quat_data.x, quat_data.y, quat_data.z, quat_data.w);
                            
                            // 创建四元数消息并发布
                            auto quat_msg = std::make_unique<geometry_msgs::msg::QuaternionStamped>();
                            quat_msg->header.stamp = this->now();
                            quat_msg->header.frame_id = "imu_link";
                            quat_msg->quaternion.x = quat_data.x;
                            quat_msg->quaternion.y = quat_data.y;
                            quat_msg->quaternion.z = quat_data.z;
                            quat_msg->quaternion.w = quat_data.w;
                            
                            // 发布四元数消息
                            quaternion_publisher_->publish(*quat_msg);
                        } else {
                            RCLCPP_WARN(this->get_logger(), 
                                      "四元数帧数据长度不足: %zu (期望 >= %zu)",
                                      frame_len - 4, sizeof(QuaternionData) + 3);
                        }
                    }
                    // 处理加速度数据
                    else if (data_id == acceleration_data_id_) {
                        // 检查帧长度至少包含头部(2字节)+数据ID(2字节)+加速度(12字节)+校验(1字节)+尾部(2字节)
                        if (frame_len >= 4 + sizeof(AccelerationData) + 3) {
                            AccelerationData accel_data;
                            // 使用memcpy从数据帧提取加速度
                            memcpy(&accel_data, &frame[4], sizeof(AccelerationData));
                            
                            // 打印加速度数据
                            // RCLCPP_INFO(this->get_logger(), 
                            //            "加速度数据: x=%.4f, y=%.4f, z=%.4f",
                            //            accel_data.x, accel_data.y, accel_data.z);
                            
                            // 创建加速度消息并发布
                            auto accel_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
                            accel_msg->header.stamp = this->now();
                            accel_msg->header.frame_id = "imu_link";
                            accel_msg->vector.x = accel_data.x;
                            accel_msg->vector.y = accel_data.y;
                            accel_msg->vector.z = accel_data.z;
                            
                            // 发布加速度消息
                            acceleration_publisher_->publish(*accel_msg);
                        } else {
                            RCLCPP_WARN(this->get_logger(), 
                                      "加速度帧数据长度不足: %zu (期望 >= %zu)",
                                      frame_len - 4, sizeof(AccelerationData) + 3);
                        }
                    }
                    return;
                }
            }

            // 继续原来的处理逻辑（通过协议解析器）
            try {
                FrameType frame_type = ProtocolParser::parse_frame(frame);
                if (frame_type == FrameType::DataFrame) {
                    // 解析数据帧
                    DataFrame data_frame = ProtocolParser::parse_data(frame);
                    
                    // 检查数据ID
                    if (data_frame.data_id == quaternion_data_id_ && 
                        data_frame.data.size() >= sizeof(QuaternionData)) {
                        // 处理四元数数据
                        QuaternionData quat_data;
                        memcpy(&quat_data, data_frame.data.data(), sizeof(QuaternionData));
                        
                        // 创建四元数消息并发布
                        auto quat_msg = std::make_unique<geometry_msgs::msg::QuaternionStamped>();
                        quat_msg->header.stamp = this->now();
                        quat_msg->header.frame_id = "imu_link";
                        quat_msg->quaternion.x = quat_data.x;
                        quat_msg->quaternion.y = quat_data.y;
                        quat_msg->quaternion.z = quat_data.z;
                        quat_msg->quaternion.w = quat_data.w;
                        
                        // 打印四元数数据
                        // RCLCPP_INFO(this->get_logger(), 
                        //            "四元数数据(标准帧): x=%.4f, y=%.4f, z=%.4f, w=%.4f",
                        //            quat_data.x, quat_data.y, quat_data.z, quat_data.w);
                        
                        // 发布四元数消息
                        quaternion_publisher_->publish(*quat_msg);
                    }
                    else if (data_frame.data_id == acceleration_data_id_ && 
                             data_frame.data.size() >= sizeof(AccelerationData)) {
                        // 处理加速度数据
                        AccelerationData accel_data;
                        memcpy(&accel_data, data_frame.data.data(), sizeof(AccelerationData));
                        
                        // 创建加速度消息并发布
                        auto accel_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
                        accel_msg->header.stamp = this->now();
                        accel_msg->header.frame_id = "imu_link";
                        accel_msg->vector.x = accel_data.x;
                        accel_msg->vector.y = accel_data.y;
                        accel_msg->vector.z = accel_data.z;
                        
                        // 打印加速度数据
                        // RCLCPP_INFO(this->get_logger(), 
                        //            "加速度数据(标准帧): x=%.4f, y=%.4f, z=%.4f",
                        //            accel_data.x, accel_data.y, accel_data.z);
                        
                        // 发布加速度消息
                        acceleration_publisher_->publish(*accel_msg);
                    }
                    else {
                        RCLCPP_DEBUG(this->get_logger(), 
                                    "收到未知数据ID: 0x%04X，数据长度: %zu",
                                    data_frame.data_id, data_frame.data.size());
                    }
                }
            } catch (const ProtocolException& e) {
                RCLCPP_ERROR(this->get_logger(), "协议解析错误: %s", e.what());
            }
        } catch (const SerialException& e) {
            // 忽略超时异常，其他异常记录
            if (std::string(e.what()) != "Read timeout") {
                RCLCPP_ERROR(this->get_logger(), "串口错误: %s", e.what());
            }
        }
    }
    
    // 成员变量
    std::unique_ptr<SerialInterface> serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acceleration_publisher_;
    int read_timeout_ms_;
    uint16_t quaternion_data_id_;
    uint16_t acceleration_data_id_;
};

} // namespace ares_comm

// 主函数
int main(int argc, char* argv[]) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点并运行
    auto node = std::make_shared<ares_comm::IMUSensorNode>();
    rclcpp::spin(node);
    
    // 清理资源
    rclcpp::shutdown();
    return 0;
} 
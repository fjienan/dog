#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "ares_comm/serial_interface.hpp"
#include "ares_comm/protocol_parser.hpp"

// ROS消息和服务
#include "ares_comm/msg/data.hpp"
#include "ares_comm/msg/error.hpp"
#include "ares_comm/srv/execute.hpp"
#include "geometry_msgs/msg/twist.hpp"  // 添加Twist消息类型

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ares_comm {

class AresCommNode : public rclcpp::Node {
public:
    AresCommNode() : Node("ares_comm_node"), request_id_counter_(0) {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("read_timeout_ms", 100);
        
        // 添加cmd_vel相关参数
        this->declare_parameter<uint16_t>("vel_func_id", 0x0001);  // 速度控制功能ID

        // 获取参数
        std::string port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        read_timeout_ms_ = this->get_parameter("read_timeout_ms").as_int();
        vel_func_id_ = this->get_parameter("vel_func_id").as_int();

        // 创建发布者  为啥要这些东西？
        data_publisher_ = this->create_publisher<ares_comm::msg::Data>("ares/data", 10);
        error_publisher_ = this->create_publisher<ares_comm::msg::Error>("ares/error", 10);

        // 创建服务
        execute_service_ = this->create_service<ares_comm::srv::Execute>(
            "ares/execute",
            std::bind(&AresCommNode::handle_execute, this, _1, _2)
        );
        
        // 创建cmd_vel订阅者
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&AresCommNode::cmd_vel_callback, this, _1));

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
            10ms, std::bind(&AresCommNode::read_serial, this));
            
        // 添加一个测试按钮，发送固定的cmd_vel (频率提高到200ms发送一次)
        test_timer_ = this->create_wall_timer(
            50ms, std::bind(&AresCommNode::send_test_cmd_vel, this));
    }
    
    ~AresCommNode() {
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

            // 先检查帧类型，如果是心跳包或同步帧就直接返回，不打印任何信息
            if (frame.size() >= 2) {
                uint16_t header = (frame[0] << 8) | frame[1];
                if (header == 0xCADE || header == 0x5A5A) {
                    // 这是心跳包或同步帧，直接跳过不处理
                    return;
                }
            }

            // 打印接收到的原始帧（十六进制格式）
            std::stringstream hex_data;

            // 先确定帧类型
            std::string frame_type_str = "未知类型";
            if (frame.size() >= 2) {
                uint16_t header = (frame[0] << 8) | frame[1];
                switch (header) {
                    case 0xCAFE: frame_type_str = "执行帧"; break;
                    case 0xC0DE: frame_type_str = "响应帧"; break;
                    case 0x5A5A: frame_type_str = "同步帧"; break;
                    case 0xCADE: frame_type_str = "心跳包"; break;
                    default: frame_type_str = "未知类型"; break;
                }
            }

            hex_data << "接收到" << frame_type_str << ": ";
            for (uint8_t byte : frame) {
                hex_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", hex_data.str().c_str());

            // 解析帧类型
            FrameType frame_type = ProtocolParser::parse_frame(frame);

            switch (frame_type) {
                case FrameType::DataFrame: {
                    // 同步帧应该在前面的检查中就已经过滤掉了
                    break;
                }
                case FrameType::ErrorFrame: {
                    // 这是心跳包，已经在前面过滤了，不应该走到这里
                    break;
                }
                case FrameType::ResponseFrame: {
                    // 处理响应帧
                    ResponseFrame response_frame = ProtocolParser::parse_response(frame);
                    
                    RCLCPP_INFO(this->get_logger(), "收到响应帧: Func ID=0x%04X, Value=%d, RequestID=%d, Success=%s", 
                               response_frame.func_id, response_frame.value, response_frame.request_id,
                               response_frame.success ? "成功" : "失败");
                    
                    process_response(response_frame);
                    break;
                }
                default:
                    RCLCPP_INFO(this->get_logger(), "接收到未知帧类型");
                    break;
            }
        } catch (const SerialException& e) {
            // 忽略超时异常，其他异常记录
            if (std::string(e.what()) != "Read timeout") {
                RCLCPP_ERROR(this->get_logger(), "串口错误: %s", e.what());
            }
        } catch (const ProtocolException& e) {
            RCLCPP_ERROR(this->get_logger(), "协议错误: %s", e.what());
        }
    }

    void handle_execute(
        const std::shared_ptr<ares_comm::srv::Execute::Request> request,
        std::shared_ptr<ares_comm::srv::Execute::Response> response)
    {
        if (!serial_ || !serial_->is_open()) {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "串口未打开");
            return;
        }

        try {
            // 创建执行帧
            ExecuteFrame frame;
            frame.func_id = request->func_id;
            frame.arg1 = request->arg1;
            frame.arg2 = request->arg2;
            frame.arg3 = request->arg3;
            frame.request_id = request->request_id;

            // 序列化并发送
            std::vector<uint8_t> data = ProtocolParser::serialize_execute(frame);
            
            // 打印发送的帧（十六进制格式）
            std::stringstream hex_data;
            hex_data << "发送执行帧: ";
            for (uint8_t byte : data) {
                hex_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", hex_data.str().c_str());
            
            serial_->write(data);

            // 等待响应 (延迟确保下位机有足够时间处理)
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            
            // 尝试读取响应 - 直接使用read_frame来确保帧对齐
            bool got_response = false;
            int retries = 3;  // 最多尝试3次
            
            while (retries-- > 0 && !got_response) {
                try {
                    std::vector<uint8_t> frame_data = serial_->read_frame();
                    
                    if (frame_data.empty()) {
                        continue;
                    }
                    
                    // 打印接收到的帧
                    std::stringstream recv_hex;
                    // 先确定帧类型
                    std::string frame_type_str = "未知类型";
                    if (frame_data.size() >= 2) {
                        uint16_t header = (frame_data[0] << 8) | frame_data[1];
                        switch (header) {
                            case 0xCAFE: frame_type_str = "执行帧"; break;
                            case 0xC0DE: frame_type_str = "响应帧"; break;
                            case 0x5A5A: frame_type_str = "同步帧"; break;
                            case 0xCADE: frame_type_str = "心跳包"; break;
                            default: frame_type_str = "未知类型"; break;
                        }
                    }
                    recv_hex << "接收到" << frame_type_str << ": ";
                    for (uint8_t byte : frame_data) {
                        recv_hex << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                    }
                    RCLCPP_INFO(this->get_logger(), "%s", recv_hex.str().c_str());
                    
                    // 解析帧类型
                    FrameType frame_type = ProtocolParser::parse_frame(frame_data);
                    
                    switch (frame_type) {
                        case FrameType::ResponseFrame: {
                            // 正常响应帧
                            ResponseFrame resp = ProtocolParser::parse_response(frame_data);
                            
                            // 检查请求ID是否匹配
                            if (resp.request_id == frame.request_id) {
                                response->success = true;
                                response->value = resp.value;
                                RCLCPP_INFO(this->get_logger(), "成功接收到响应");
                                got_response = true;
                            } else {
                                RCLCPP_WARN(this->get_logger(), "响应ID不匹配: 期望=%d, 实际=%d", 
                                          frame.request_id, resp.request_id);
                            }
                            break;
                        }
                        
                        case FrameType::ErrorFrame: {
                            // 错误帧
                            ErrorFrame err = ProtocolParser::parse_error(frame_data);
                            
                            if (err.request_id == frame.request_id) {
                                response->success = false;
                                RCLCPP_ERROR(this->get_logger(), "收到错误帧，错误码: 0x%04X", 
                                           err.error_code);
                                got_response = true;
                            } else {
                                RCLCPP_WARN(this->get_logger(), "错误帧ID不匹配: 期望=%d, 实际=%d", 
                                          frame.request_id, err.request_id);
                            }
                            break;
                        }
                        
                        case FrameType::DataFrame:
                            // 同步帧，不是我们期望的响应类型，但不打印日志
                            break;
                            
                        default:
                            // 其他类型帧，不打印日志
                            break;
                    }
                } catch (const SerialException& e) {
                    // 读超时或格式错误，继续尝试
                    RCLCPP_DEBUG(this->get_logger(), "读取响应时发生错误: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                } catch (const ProtocolException& e) {
                    // 协议错误，继续尝试
                    RCLCPP_DEBUG(this->get_logger(), "协议错误: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
            
            if (!got_response) {
                response->success = false;
                RCLCPP_WARN(this->get_logger(), "请求ID %d 未收到响应", frame.request_id);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "执行命令出错: %s", e.what());
            response->success = false;
        }
    }

    void process_response(const ResponseFrame& frame) {
        // 不需要额外的日志，因为在read_serial中已经添加了响应日志
        // 这里可以添加更多处理逻辑，例如匹配请求ID等
    }
    
    // cmd_vel回调函数
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!serial_ || !serial_->is_open()) {
            RCLCPP_INFO(this->get_logger(), "串口未打开，无法发送cmd_vel");
            return;
        }
        
        try {
            // 将Twist消息转换为下位机可理解的格式
            // 这里假设下位机需要三个参数：线速度x，线速度y，角速度z
            // // 并且需要将浮点数转换为整数（通过缩放）
            // int32_t linear_x = static_cast<int32_t>(msg->linear.x * linear_scale_);
            // int32_t linear_y = static_cast<int32_t>(msg->linear.y * linear_scale_);
            // int32_t angular_z = static_cast<int32_t>(msg->angular.z * angular_scale_);
            
            // 创建执行帧
            ExecuteFrame frame;
            frame.func_id = vel_func_id_;     // 速度控制的功能ID
            frame.arg1 = msg->linear.x;
            frame.arg2 = msg->linear.y;
            frame.arg3 = msg->angular.z;
            frame.request_id = next_request_id();
            
            RCLCPP_INFO(this->get_logger(), 
                      "发送速度命令: linear_x=%.2f, linear_y=%.2f, angular_z=%.2f, RequestID=%d", 
                      msg->linear.x, msg->linear.y, msg->angular.z, frame.request_id);
            
            // 序列化并发送
            std::vector<uint8_t> data = ProtocolParser::serialize_execute(frame);
            
            // 打印发送的帧数据（十六进制格式）
            std::stringstream hex_data;
            hex_data << "发送执行帧: ";
            for (uint8_t byte : data) {
                hex_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", hex_data.str().c_str());
            
            serial_->write(data);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送cmd_vel错误: %s", e.what());
        }
    }
    
    // 发送测试cmd_vel
    void send_test_cmd_vel() {
        auto test_msg = std::make_unique<geometry_msgs::msg::Twist>();
        test_msg->linear.x = 0.1;  // 前进0.1 m/s
        test_msg->linear.y = 0.3;
        test_msg->angular.z = 0.2; // 以0.2 rad/s旋转
        
        RCLCPP_INFO(this->get_logger(), "发送测试cmd_vel: 线速度x=%.2f m/s, 角速度z=%.2f rad/s", 
                   test_msg->linear.x, test_msg->angular.z);
        
        cmd_vel_callback(std::move(test_msg));
    }
    
    // 生成下一个请求ID
    uint8_t next_request_id() {
        request_id_counter_ = (request_id_counter_ + 1) % 256;
        return request_id_counter_;
    }

    // 成员变量
    std::unique_ptr<SerialInterface> serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    rclcpp::Publisher<ares_comm::msg::Data>::SharedPtr data_publisher_;
    rclcpp::Publisher<ares_comm::msg::Error>::SharedPtr error_publisher_;
    rclcpp::Service<ares_comm::srv::Execute>::SharedPtr execute_service_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    int read_timeout_ms_;
    uint16_t vel_func_id_;
    uint8_t request_id_counter_;
};

} // namespace ares_comm

// 主函数
int main(int argc, char* argv[]) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点并运行
    auto node = std::make_shared<ares_comm::AresCommNode>();
    rclcpp::spin(node);
    
    // 清理资源
    rclcpp::shutdown();
    return 0;
} 
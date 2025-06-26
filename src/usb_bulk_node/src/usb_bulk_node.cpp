#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <geometry_msgs/msg/vector3.hpp>         // 角速度和加速度
#include <geometry_msgs/msg/quaternion.hpp>      // 四元数
#include <geometry_msgs/msg/point.hpp>           // 五点坐标

#include "ares_protocol.hpp"
#include <iostream>
#include <vector>
#include <chrono> // for sleep
#include <thread> // for sleep
#include <limits> // Required for numeric_limits
#include <ios>    // Required for streamsize

#define ACCEL_NODE_ID 0x1001
#define GYRO_NODE_ID 0x1002
#define QUATERNION_NODE_ID 0x1003
#define FIVE_FUNC_ID 0x4004


class SubscriberNode : public rclcpp::Node {
public:
    ares::Protocol proto;
    std::chrono::_V2::steady_clock::time_point last_update = std::chrono::steady_clock::now();
    int package_published = 0;

    void sync_handler(uint16_t data_id, const uint8_t* data, size_t len) {
        if (data == nullptr) {
            std::cerr << "Error: Received null data pointer" << std::endl;
            return;
        }

        package_published++;

        ms = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
        );

        // std::cout << ms.count() << "[Callback] Sync received: DataID=" << data_id << ", Len=" << len << std::endl;

        if(std::chrono::steady_clock::now() - last_update > std::chrono::seconds(1)) {
            last_update = std::chrono::steady_clock::now();
            std::cout << "Updating..." << package_published << " packages published." << std::endl;
            // std::cout << "DataID: " << data_id << ", Len: " << len << std::endl;
        }
    
        switch(data_id) {
            case ACCEL_NODE_ID: {
                // 假设数据格式为三个float32（12字节）
                if (len < 12) {
                    std::cerr << "Error: Insufficient data length for acceleration" << std::endl;
                    return;
                }
                const float* float_data = reinterpret_cast<const float*>(data);
                acceleration.x = float_data[0];
                acceleration.y = float_data[1];
                acceleration.z = float_data[2];
                acceleration_flag = true;
                break;
            }
    
            case GYRO_NODE_ID: {
                // 假设数据格式为三个float32（12字节）
                if (len < 12) {
                    std::cerr << "Error: Insufficient data length for angular velocity" << std::endl;
                    return;
                }
                const float* float_data = reinterpret_cast<const float*>(data);
                angular_vel.x = float_data[0];
                angular_vel.y = float_data[1];
                angular_vel.z = float_data[2];
                angular_vel_flag = true;
                break;
            }
    
            case QUATERNION_NODE_ID: {
                // 假设数据格式为四个float32（16字节）
                if (len < 16) {
                    std::cerr << "Error: Insufficient data length for quaternion" << std::endl;
                    return;
                }
                const float* float_data = reinterpret_cast<const float*>(data);
                quaternion.x = float_data[0];
                quaternion.y = float_data[1];
                quaternion.z = float_data[2];
                quaternion.w = float_data[3];
                quaternion_flag = true;
                break;
            }
    
            default:
                std::cerr << "Unknown DataID: " << data_id << std::endl;
        }
    }

    SubscriberNode() : Node("receive_remote_control") {
        int err = this->proto.connect();

        if(err != 1) {
            std::cerr << "Failed to connect to USB device: " << err << std::endl;
            return;
        }
        
        this->proto.register_sync_callback(
            std::bind(&SubscriberNode::sync_handler, this, 
                      std::placeholders::_1, 
                      std::placeholders::_2, 
                      std::placeholders::_3));
        
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SubscriberNode::topic_callback, this, std::placeholders::_1));
		five_func_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
			"/five_func", 10, std::bind(&SubscriberNode::five_func_callback, this, std::placeholders::_1));
        
        // 创建发布者
        angular_vel_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/imu/angular_velocity", 10);
        acceleration_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/imu/acceleration", 10);
        quaternion_pub_ = create_publisher<geometry_msgs::msg::Quaternion>("/imu/quaternion", 10);
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&SubscriberNode::timer_callback, this)
        );
        
        angular_vel = geometry_msgs::msg::Vector3();
        acceleration = geometry_msgs::msg::Vector3();
        quaternion = geometry_msgs::msg::Quaternion();
    }

private:
    std::chrono::milliseconds ms;
    std::chrono::steady_clock::time_point prev_exec;

	void five_func_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
		// Convert double to float, then reinterpret as uint32_t
        float float_arg1 = static_cast<float>(msg->x);
        float float_arg2 = -static_cast<float>(msg->y);

        // Reinterpret the float's binary representation as uint32_t
        uint32_t arg1 = *reinterpret_cast<uint32_t*>(&float_arg1);
        uint32_t arg2 = *reinterpret_cast<uint32_t*>(&float_arg2);

		// Send the command
		int err = this->proto.send_exec(FIVE_FUNC_ID, arg1, arg2, 0xFFFF, 0x01);
		if (!err) {
			this->proto.connect();
			std::cerr << "Failed to send Exec command: " << err << std::endl;
		}
	}

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Convert double to float, then reinterpret as uint32_t
        float float_arg1 = static_cast<float>(msg->linear.x);
        float float_arg2 = -static_cast<float>(msg->linear.y);
        float float_arg3 = -static_cast<float>(msg->angular.x);

        // Reinterpret the float's binary representation as uint32_t
        uint32_t arg1 = *reinterpret_cast<uint32_t*>(&float_arg1);
        uint32_t arg2 = *reinterpret_cast<uint32_t*>(&float_arg2);
        uint32_t arg3 = *reinterpret_cast<uint32_t*>(&float_arg3);

        // Send the command
        int err = this->proto.send_exec(0x1, arg1, arg2, arg3, 0x01);
        if (!err) {
            this->proto.connect();
            std::cerr << "Failed to send Exec command: " << err << std::endl;
        }

        // Debug output
        printf("Original values (double): linear.x: %f, linear.y: %f, angular.x: %f\n", 
            msg->linear.x, msg->linear.y, msg->angular.x);
        printf("Float values: arg1_float: %f, arg2_float: %f, arg3_float: %f\n",
            float_arg1, float_arg2, float_arg3);
        printf("Binary representation (uint32_t): Arg1: 0x%08X, Arg2: 0x%08X, Arg3: 0x%08X\n", 
            arg1, arg2, arg3);

        prev_exec = std::chrono::steady_clock::now();
    }

    void timer_callback() {
        // If prev_exec is more than 1 second ago
        if (std::chrono::steady_clock::now() - prev_exec > std::chrono::seconds(1)) {
            std::cerr << "No data received in the last second." << std::endl;
            float farg1 = 0.0f;
            float farg2 = 0.0f;
            float farg3 = 0.0f;
            // Send the command
            uint32_t uarg1 = *reinterpret_cast<uint32_t*>(&farg1);
            uint32_t uarg2 = *reinterpret_cast<uint32_t*>(&farg2);
            uint32_t uarg3 = *reinterpret_cast<uint32_t*>(&farg3);
            int err = this->proto.send_exec(0x1, uarg1, uarg2, uarg3, 0x01);
            if (!err) {
                this->proto.connect();
                std::cerr << "Failed to send Exec command: " << err << std::endl;
            }
        }
        // 发布角速度
        if(angular_vel_flag){
            angular_vel_pub_->publish(angular_vel);
        }

        // 发布加速度
        if(acceleration_flag){
            acceleration_pub_->publish(acceleration);
        }

        // 发布四元数
        if(quaternion_flag){
            quaternion_pub_->publish(quaternion);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr five_func_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acceleration_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_pub_;

    geometry_msgs::msg::Vector3 angular_vel;
    geometry_msgs::msg::Vector3 acceleration;
    geometry_msgs::msg::Quaternion quaternion;
    geometry_msgs::msg::Point five_coordinate;

    bool angular_vel_flag = false;
    bool acceleration_flag = false;
    bool quaternion_flag = false;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}

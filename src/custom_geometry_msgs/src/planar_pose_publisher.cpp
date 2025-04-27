#include "rclcpp/rclcpp.hpp"
#include "custom_geometry_msgs/msg/planar_pose.hpp"
#include "geometry_msgs/msg/point.hpp"

class PlanarPublisher : public rclcpp::Node {
public:
  PlanarPublisher() : Node("planar_publisher") {
    publisher_ = this->create_publisher<custom_geometry_msgs::msg::PlanarPose>("planar_pose", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        auto msg = custom_geometry_msgs::msg::PlanarPose();
        msg.position.x = 1.0;  // 设置 geometry_msgs/Point 的 x 坐标
        msg.position.y = 2.0;  // 设置 y 坐标
        msg.position.z = 0.0;  // z 固定为 0（二维平面）
        msg.theta = 0.5;       // 方向角
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: x=%.2f, y=%.2f, theta=%.2f", 
                     msg.position.x, msg.position.y, msg.theta);
      }
    );
  }

private:
  rclcpp::Publisher<custom_geometry_msgs::msg::PlanarPose>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarPublisher>());
  rclcpp::shutdown();
  return 0;
}
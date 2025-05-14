/**
 * ares_control_node.cpp  ‒  预测性减速 + 限加速度控制 + 朝向控制
 *
 * 订阅：
 *   /Odometry    nav_msgs/msg/Odometry      当前里程计
 *   /goal_pose   geometry_msgs/msg/PoseStamped  目标点
 *
 * 发布：
 *   /cmd_vel     geometry_msgs/msg/Twist    速度命令
 *
 * 发送频率 50 Hz，线速度≤0.5 m/s，加速度≤0.4 m/s²
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <mutex>
#include <tf2/utils.h>  // 添加tf2工具用于四元数转欧拉角

using std::placeholders::_1;
using geometry_msgs::msg::Twist;

/* ───────────── 常量参数 ───────────── */
constexpr double kCtrlFreq        = 20.0;           // Hz
constexpr double kDt              = 1.0 / kCtrlFreq;
constexpr double kVMax            = 0.5;            // m/s   最高线速度
constexpr double kAMax            = 0.4;            // m/s²  线加速度限幅
constexpr double kArriveThresh    = 0.1;           // m     认为到点的距离
constexpr double kWMax            = 0.5;            // rad/s 最大角速度
constexpr double kP_Yaw           = 1.0;            // 朝向控制比例增益

/* ───────────── 控制节点类 ───────────── */
class AresControlNode : public rclcpp::Node
{
public:
    AresControlNode() : Node("ares_control_node")
    {
        /* 订阅 */
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&AresControlNode::odomCb, this, _1));

        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AresControlNode::goalCb, this, _1));

        /* 发布 - 改为发布 cmd_vel 消息 */
        pub_vel_ = this->create_publisher<Twist>("/cmd_vel", 10);

        /* 定时器 50 Hz */
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(kDt * 1000)),
            std::bind(&AresControlNode::loop, this));

        RCLCPP_INFO(this->get_logger(), "Ares-control node (predictive decel + yaw control) started");
    }

private:
    /* ───── 四元数转偏航角 ───── */
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        return tf2::getYaw(tf_q);
    }

    /* ───── 回调 ───── */
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::scoped_lock lk(mtx_);
        bias_x = 0.06;
        bias_y = -0.42;
        length = 0.42
        cur_x_ = -msg->pose.pose.position.x ;
        cur_y_ = -msg->pose.pose.position.y ;
        // 提取当前朝向
        cur_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

        // length = ;
        cur_x = cur_x - length*math.sin(cur_yaw);
        cur_y = cur_y - length*math.cos(cur_yaw);
        odom_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "Odometry + bias_y: x=%.2f, y=%.2f, yaw=%.2f", cur_x_, cur_y_, cur_yaw_);
    }

    void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::scoped_lock lk(mtx_);
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        // 提取目标朝向
        goal_yaw_ = quaternionToYaw(msg->pose.orientation);
        goal_has_orientation_ = true;  // 标记是否提供了朝向信息
        goal_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal received: x=%.2f, y=%.2f, yaw=%.2f", goal_x_, goal_y_, goal_yaw_);
    }

    /* ───── 主循环 ───── */
    void loop()
    {
        if (!odom_ok_ || !goal_ok_) return;

        double x, y, yaw, gx, gy, gyaw;
        bool has_goal_yaw;
        {
            std::scoped_lock lk(mtx_);
            x = cur_x_;  y = cur_y_;  yaw = cur_yaw_;
            gx = goal_x_; gy = goal_y_; gyaw = goal_yaw_;
            has_goal_yaw = goal_has_orientation_;
        }

        /* 1. 误差向量 */
        double dx = gx - x;
        double dy = gy - y;
        double dist = std::hypot(dx, dy);
        
        RCLCPP_INFO(this->get_logger(), "X Distance to goal: %.2f", dx);
        
        RCLCPP_INFO(this->get_logger(), "Y Distance to goal: %.2f", dy);
        // RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f", dist);
        /* 2. 到点直接停 */
        if (dist < kArriveThresh) {
            prev_v_ = 0.0;
            
            /* 2.1 到点后如果有目标朝向，继续调整朝向 */
            if (has_goal_yaw) {
                double yaw_error = gyaw - yaw;
                // 归一化到[-π, π]
                while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
                while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
                
                // 角度足够接近就停止
                if (std::abs(yaw_error) < 0.05) {
                    sendVelocity(0.0, 0.0, 0.0);
                } else {
                    // 朝向控制
                    double wz = kP_Yaw * yaw_error;
                    // 限制最大角速度
                    wz = std::clamp(wz, -kWMax, kWMax);
                    sendVelocity(0.0, 0.0, wz);
                }
            } else {
                sendVelocity(0.0, 0.0, 0.0);
            }
            return;
        }

        /* 3. 安全速度 = √(2 a_max dist) */
        double v_safe = std::sqrt(2.0 * kAMax * dist);

        /* 4. 期望速度 (不超 vmax) */
        double v_des = std::min(kVMax, v_safe);

        /* 5. 限加速度滤波 */
        double dv_lim = kAMax * kDt;
        double dv = std::clamp(v_des - prev_v_, -dv_lim, dv_lim);
        double v_cmd = prev_v_ + dv;
        prev_v_ = v_cmd;

        /* 6. 分解为 vx, vy（保持直线路径） */
        double ux = dx / dist;      // 单位向量
        double uy = dy / dist;
        double vx = v_cmd * ux;
        double vy = v_cmd * uy;

        /* 7. 计算目标方向角度（移动方向） */
        double target_yaw = std::atan2(dy, dx);  // 目标点方向的朝向
        
        /* 8. 如果提供了目标朝向，使用目标朝向，否则使用移动方向 */
        double desired_yaw = has_goal_yaw ? gyaw : target_yaw;
        
        /* 9. 计算朝向误差并生成角速度命令 */
        double yaw_error = desired_yaw - yaw;
        // 归一化到[-π, π]
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        
        // 朝向控制
        double wz = kP_Yaw * yaw_error;
        // 限制最大角速度
        wz = std::clamp(wz, -kWMax, kWMax);

        /* 发送速度命令 */
        sendVelocity(vx, vy, wz);
        RCLCPP_INFO(this->get_logger(), "Sending velocity command: vx=%.2f, vy=%.2f, wz=%.2f", vx, vy, wz);
        // sendVelocity(0.1,0,0);
    }

    /* ───── 发送速度命令 ───── */
    void sendVelocity(double vx, double vy, double wz)
    {
        auto twist_msg = std::make_unique<Twist>();
        twist_msg->linear.x = vx;
        twist_msg->linear.y = vy;
        twist_msg->angular.z = wz;
        
        pub_vel_->publish(std::move(twist_msg));
    }

    /* ───── 成员 ───── */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Publisher<Twist>::SharedPtr                            pub_vel_;
    rclcpp::TimerBase::SharedPtr                                  timer_;

    std::mutex mtx_;
    double cur_x_{}, cur_y_{}, cur_yaw_{};
    double goal_x_{}, goal_y_{}, goal_yaw_{};
    bool   odom_ok_{false}, goal_ok_{false}, goal_has_orientation_{false};

    /* 速度记忆（用于限加速度） */
    double prev_v_{0.0};
};

/* ───────────── main ───────────── */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AresControlNode>());
    rclcpp::shutdown();
    return 0;
}
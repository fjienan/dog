#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import Float32
import math
import time

class ObstaclePredictorNode(Node):
    def __init__(self):
        super().__init__('obstacle_predictor_node')
        
        # 参数设置
        self.declare_parameters(
            namespace='',
            parameters=[
                ('prediction_horizon', 5),
                ('dt', 0.1),
                ('max_expected_obstacle_speed', 1.0),
            ]
        )
        
        # 获取参数
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.dt = self.get_parameter('dt').value
        self.max_expected_obstacle_speed = self.get_parameter('max_expected_obstacle_speed').value
        
        # 存储障碍物历史数据
        self.obstacle_history = {}
        self.obstacle_clusters = []
        
        # 创建发布者和订阅者
        self.obstacle_predictions_pub = self.create_publisher(
            PoseArray,
            'obstacle_predictions',
            10
        )
        
        self.obstacle_positions_sub = self.create_subscription(
            PoseArray,
            'obstacle_positions',
            self.obstacle_positions_callback,
            10
        )
        
        self.get_logger().info('障碍物预测节点已启动')

    def obstacle_positions_callback(self, msg):
        """处理接收到的障碍物位置数据"""
        # 更新障碍物聚类数据
        self.obstacle_clusters = []
        for i, pose in enumerate(msg.poses):
            self.obstacle_clusters.append({
                'id': i,
                'center': [pose.position.x, pose.position.y]
            })
        
        # 预测障碍物轨迹
        predicted_paths = self.predict_obstacle_positions()
        
        # 发布预测结果
        self.publish_predictions(predicted_paths)

    def predict_obstacle_positions(self):
        """预测障碍物在MPC预测步数内的路径"""
        predicted_obstacle_paths = []

        for cluster in self.obstacle_clusters:
            obstacle_id = cluster['id']
            current_pos = np.array(cluster['center'])
            predicted_path = [current_pos.tolist()]

            # 默认速度为零
            velocity_estimate = np.array([0.0, 0.0])

            # 如果有足够的历史数据，估算速度
            if obstacle_id in self.obstacle_history and len(self.obstacle_history[obstacle_id]) >= 2:
                history = self.obstacle_history[obstacle_id]
                pos_latest = np.array(history[-1])
                pos_previous = np.array(history[-2])

                history_time_diff_approx = self.dt

                if history_time_diff_approx > 1e-9:
                    velocity_estimate = (pos_latest - pos_previous) / history_time_diff_approx

                # 限制速度估计值
                current_est_speed = np.linalg.norm(velocity_estimate)
                if current_est_speed > self.max_expected_obstacle_speed:
                    if current_est_speed > 1e-9:
                        velocity_estimate = (velocity_estimate / current_est_speed) * self.max_expected_obstacle_speed
                    else:
                        velocity_estimate = np.array([0.0, 0.0])

            # 预测未来位置
            current_predicted_pos = current_pos.copy()
            for i in range(self.prediction_horizon):
                current_predicted_pos = current_predicted_pos + velocity_estimate * self.dt
                predicted_path.append(current_predicted_pos.tolist())

            predicted_obstacle_paths.append(predicted_path)
            
            # 更新历史数据
            if obstacle_id not in self.obstacle_history:
                self.obstacle_history[obstacle_id] = []
            self.obstacle_history[obstacle_id].append(current_pos.tolist())
            
            # 限制历史数据长度
            if len(self.obstacle_history[obstacle_id]) > 10:
                self.obstacle_history[obstacle_id].pop(0)

        return predicted_obstacle_paths

    def publish_predictions(self, predicted_paths):
        """发布预测结果"""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for path in predicted_paths:
            for pos in path:
                pose = Point()
                pose.x = float(pos[0])
                pose.y = float(pos[1])
                pose.z = 0.0
                msg.poses.append(pose)

        self.obstacle_predictions_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
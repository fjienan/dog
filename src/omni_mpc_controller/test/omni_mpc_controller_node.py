#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# Add backward compatibility for transforms3d library
if not hasattr(np, 'float'):
    np.float = float  # Monkey patch numpy to add back the deprecated alias

from sklearn.cluster import DBSCAN
from scipy.optimize import minimize, linear_sum_assignment
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import ConvexHull


class OmniMPCController(Node):
    def __init__(self):
        super().__init__('omni_mpc_controller')
        
        # 发布器初始化
        self.marker_pub = self.create_publisher(Marker, '/obstacles', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/obstacle_history', 10)
        self.trajectory_pub = self.create_publisher(MarkerArray, '/mpc_trajectory', 10)
        
        self.previous_cluster_count = -1
        self.previous_obstacles = []
        
        # 时间步长初始化
        self.dt = 0.05  # 默认20Hz控制率的时间步长

        # 声明参数
        self.declare_parameters(namespace='',
            parameters=[
                ('control_rate', 20.0),
                ('max_speed', 2.0),
                ('max_accel', 2.0),
                ('safety_radius', 0.1),
                ('manual_weight', 0.5),            # 修改：增加手动控制权重(0.3->0.5)
                ('dbscan_eps', 0.8),
                ('dbscan_min_samples', 3),
                ('weight_goal', 1.0),              # 目标点权重
                ('weight_obstacle', 200.0),        # 修改：降低障碍物权重(500->200)
                ('weight_accel', 0.1),             # 加速度平滑权重
                ('prediction_horizon', 5),         # 预测步数
                ('obstacle_radius', 0.8),          # 修改：减小障碍物半径(1.0->0.8)
                ('obstacle_avoid_factor', 1.2),    # 新增：避障距离系数，表示多少倍的安全距离开始避障
                ('boundary_safe_distance', 1.5)    # 边界安全距离
            ])
        
        # 初始化参数
        self.control_rate = self.get_parameter('control_rate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_accel = self.get_parameter('max_accel').value
        self.manual_weight = self.get_parameter('manual_weight').value
        self.safety_radius = self.get_parameter('safety_radius').value
        self.weight_goal = self.get_parameter('weight_goal').value
        self.weight_obstacle = self.get_parameter('weight_obstacle').value
        self.weight_accel = self.get_parameter('weight_accel').value
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.obstacle_avoid_factor = self.get_parameter('obstacle_avoid_factor').value
        self.boundary_safe_distance = self.get_parameter('boundary_safe_distance').value
        self.dt = 1.0 / self.control_rate
        
        # 车辆状态 (全向轮需要x,y速度)
        self.pose = np.zeros(3)  # x, y, theta
        self.velocity = np.zeros(2)  # vx, vy
        self.obstacles = []
        self.obstacle_clusters = []  # 障碍物轨迹跟踪
        self.obstacle_history = {}   # 障碍物历史位置
        self.goal = None
        self.manual_cmd = np.zeros(2)  # [vx, vy]
        
        # 创建ROS接口
        self.cmd_pub = self.create_publisher(Twist, '/ally/robot1/cmd_vel', 10)
        self.create_subscription(Odometry, '/ally/robot1/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/ally/robot1/scan2', self.laser_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Twist, '/manual_cmd', self.manual_callback, 10)
        
        # 控制定时器
        self.create_timer(self.dt, self.control_cycle)
        
        # MPC轨迹预测
        self.mpc_trajectory = []  # 存储MPC预测轨迹
        
        self.get_logger().info("全向轮MPC控制器已启动 - 避障距离已优化")

    def odom_callback(self, msg):
        """处理全向轮里程计数据"""
        # 位置
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        
        # 航向角
        q = msg.pose.pose.orientation
        self.pose[2] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        
        # 线速度 (全向轮需要x,y方向速度)
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.linear.y

    def publish_obstacle_marker(self, center, marker_id):
        """发布障碍物可视化标记"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = self.obstacle_radius * 2
        marker.scale.y = self.obstacle_radius * 2
        marker.scale.z = 1.0
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = 0.5
        
        self.marker_pub.publish(marker)
    
    def publish_obstacle_history(self):
        """发布障碍物历史轨迹"""
        marker_array = MarkerArray()
        
        for obs_id, history in self.obstacle_history.items():
            if len(history) < 2:
                continue
                
            # 创建历史轨迹线
            line_marker = Marker()
            line_marker.header.frame_id = "odom"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "obstacle_history"
            line_marker.id = obs_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # 线宽
            line_marker.color.a = 0.7
            line_marker.color.r = 0.0
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0
            
            # 添加历史点
            for point in history:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.1
                line_marker.points.append(p)
            
            marker_array.markers.append(line_marker)
        
        self.marker_array_pub.publish(marker_array)
    
    def publish_mpc_trajectory(self, trajectory):
        """发布MPC预测轨迹"""
        if not trajectory:
            return
            
        marker_array = MarkerArray()
        
        # 创建轨迹线
        line_marker = Marker()
        line_marker.header.frame_id = "odom"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "mpc_trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1  # 线宽
        line_marker.color.a = 1.0
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        
        # 添加当前位置
        p_start = Point()
        p_start.x = self.pose[0]
        p_start.y = self.pose[1]
        p_start.z = 0.1
        line_marker.points.append(p_start)
        
        # 添加轨迹点
        for point in trajectory:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.1
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # 创建轨迹点标记
        for i, point in enumerate(trajectory):
            point_marker = Marker()
            point_marker.header.frame_id = "odom"
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = "mpc_points"
            point_marker.id = i+1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.scale.x = 0.2
            point_marker.scale.y = 0.2
            point_marker.scale.z = 0.2
            point_marker.color.a = 1.0
            point_marker.color.r = 0.0
            point_marker.color.g = 0.7
            point_marker.color.b = 0.3
            point_marker.pose.position.x = point[0]
            point_marker.pose.position.y = point[1]
            point_marker.pose.position.z = 0.1
            
            marker_array.markers.append(point_marker)
        
        self.trajectory_pub.publish(marker_array)

    def laser_callback(self, msg):
        """处理2D激光雷达数据并生成障碍物"""
        points = []
        robot_x, robot_y, robot_theta = self.pose

        # 转换为全局坐标系
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                local_angle = msg.angle_min + i * msg.angle_increment
                global_angle = local_angle + robot_theta
                x = robot_x + r * np.cos(global_angle)
                y = robot_y + r * np.sin(global_angle)
                points.append([x, y])
        
        # 初始化聚类相关变量
        current_cluster_count = 0
        unique_labels = set()
        clusters = []
        self.obstacles = []
        
        # DBSCAN聚类
        if points:
            dbscan = DBSCAN(eps=self.get_parameter('dbscan_eps').value, 
                           min_samples=self.get_parameter('dbscan_min_samples').value)
            clusters = dbscan.fit_predict(points)
            unique_labels = set(clusters)
            unique_labels.discard(-1)  # 移除噪声点
            current_cluster_count = len(unique_labels)
            
            # 无论是否触发更新条件，都要计算障碍物中心
            for label in unique_labels:
                cluster_points = np.array([points[i] for i, l in enumerate(clusters) if l == label])
                obstacle_center = self.calculate_obstacle_center(cluster_points, robot_x, robot_y)
                self.obstacles.append(obstacle_center)  # 关键！始终填充障碍物列表

        # 位置变化检测（必须在聚类之后）
        position_changed = False
        self.get_logger().debug(f"检测到障碍物: {len(self.obstacles)}")

        if len(self.obstacles) == len(self.previous_obstacles) and len(self.obstacles) > 0:
            distances = [np.linalg.norm(new-old) for new, old in zip(self.obstacles, self.previous_obstacles)]
            position_changed = any(d > 0.1 for d in distances)

        # 更新条件判断
        if current_cluster_count != self.previous_cluster_count or position_changed:
            # 删除旧标记
            delete_marker = Marker()
            delete_marker.header.frame_id = "odom"
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.action = Marker.DELETEALL
            delete_marker.ns = "obstacles"
            self.marker_pub.publish(delete_marker)

            # 生成新标记
            if current_cluster_count > 0:
                for idx, obstacle_center in enumerate(self.obstacles):
                    self.publish_obstacle_marker(obstacle_center, marker_id=idx)
                    # 输出障碍物位置用于调试
                    dist = np.linalg.norm(obstacle_center - self.pose[:2])
                    self.get_logger().info(f"障碍物{idx} 距离: {dist:.2f}m 位置: [{obstacle_center[0]:.2f}, {obstacle_center[1]:.2f}]")

            # 更新状态记录
            self.previous_cluster_count = current_cluster_count
            self.previous_obstacles = self.obstacles.copy()
            self.get_logger().info(f"障碍物数量变化：{self.previous_cluster_count}")
        
        # 更新障碍物跟踪
        if len(self.obstacles) > 0:
            self.track_obstacles()
            # 发布障碍物历史轨迹
            self.publish_obstacle_history()
        else:
            self.obstacle_clusters = []

    def calculate_obstacle_center(self, cluster_points, robot_x, robot_y):
        """计算位于激光点后方的障碍物中心"""
        # 方法1：射线反向投影法
        # 计算所有点相对于机器人的平均方向
        avg_angle = np.arctan2(
            (cluster_points[:,1]-robot_y).mean(),
            (cluster_points[:,0]-robot_x).mean()
        )
        
        # 计算平均距离并增加安全余量
        avg_distance = np.mean(np.linalg.norm(cluster_points - [robot_x, robot_y], axis=1))
        safety_margin = 0.3  # 根据障碍物半径调整
        
        # 障碍物中心位于射线延长线上（更远离机器人）
        obstacle_center = [
            robot_x + (avg_distance + safety_margin) * np.cos(avg_angle),
            robot_y + (avg_distance + safety_margin) * np.sin(avg_angle)
        ]
        
        return np.array(obstacle_center)    

    def track_obstacles(self):
        """用于障碍物跟踪和速度估计的函数"""
        if not hasattr(self, 'obstacle_clusters') or not self.obstacle_clusters:
            # 初始化障碍物跟踪
            self.obstacle_clusters = []
            for i, obs in enumerate(self.obstacles):
                self.obstacle_clusters.append({
                    'id': i,
                    'center': obs,
                    'velocity': np.zeros(2),
                    'last_seen': self.get_clock().now().nanoseconds
                })
                self.obstacle_history[i] = [obs]
            return
            
        if not self.obstacles:
            self.obstacle_clusters = []
            return
            
        # 创建代价矩阵（考虑距离）
        cost_matrix = np.zeros((len(self.obstacle_clusters), len(self.obstacles)))
        for i, prev in enumerate(self.obstacle_clusters):
            for j, curr in enumerate(self.obstacles):
                # 以距离作为主要代价
                distance_cost = np.linalg.norm(prev['center'] - curr)
                cost_matrix[i,j] = distance_cost
        
        # 使用匈牙利算法进行匹配
        try:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            # 更新障碍物信息
            new_clusters = []
            current_time = self.get_clock().now().nanoseconds
            used_obs_indices = set()
            
            # 处理匹配的障碍物
            for i, j in zip(row_ind, col_ind):
                # 只接受合理距离内的匹配
                if cost_matrix[i, j] > 2.0:  # 如果匹配距离太远，视为新障碍物
                    continue
                    
                prev = self.obstacle_clusters[i]
                curr = self.obstacles[j]
                used_obs_indices.add(j)
                
                # 计算速度（平滑过滤）
                velocity = (curr - prev['center']) / self.dt
                filtered_velocity = 0.7 * prev['velocity'] + 0.3 * velocity
                
                # 更新障碍物历史位置
                obstacle_id = prev['id']
                if obstacle_id not in self.obstacle_history:
                    self.obstacle_history[obstacle_id] = []
                self.obstacle_history[obstacle_id].append(curr)
                # 只保留最近10个位置
                if len(self.obstacle_history[obstacle_id]) > 10:
                    self.obstacle_history[obstacle_id] = self.obstacle_history[obstacle_id][-10:]
                
                new_clusters.append({
                    'id': obstacle_id,
                    'center': curr,
                    'velocity': filtered_velocity,
                    'last_seen': current_time
                })
            
            # 添加新出现的障碍物
            for j in range(len(self.obstacles)):
                if j not in used_obs_indices:
                    new_id = max([c['id'] for c in self.obstacle_clusters], default=-1) + 1
                    new_clusters.append({
                        'id': new_id,
                        'center': self.obstacles[j],
                        'velocity': np.zeros(2),
                        'last_seen': current_time
                    })
                    self.obstacle_history[new_id] = [self.obstacles[j]]
            
            self.obstacle_clusters = new_clusters
            
        except Exception as e:
            self.get_logger().warn(f"障碍物匹配错误: {str(e)}")
    
    def predict_obstacle_positions(self):
        """预测障碍物未来位置"""
        predicted_positions = []
        
        for cluster in self.obstacle_clusters:
            obstacle_id = cluster['id']
            
            if obstacle_id in self.obstacle_history and len(self.obstacle_history[obstacle_id]) > 1:
                # 使用历史位置预测障碍物速度
                history = self.obstacle_history[obstacle_id]
                pos1 = np.array(history[-1])
                pos2 = np.array(history[-2])
                
                # 使用时间差计算速度
                velocity = (pos1 - pos2) / self.dt
                
                # 使用已计算的存储速度
                if 'velocity' in cluster and np.linalg.norm(cluster['velocity']) > 0.01:
                    velocity = cluster['velocity']
                
                # 预测障碍物未来位置
                predicted_position = pos1 + velocity * self.prediction_horizon * self.dt
                predicted_positions.append(predicted_position)
                
                # 调试输出
                speed = np.linalg.norm(velocity)
                if speed > 0.1:  # 只有速度明显时才输出
                    self.get_logger().debug(f"障碍物ID:{obstacle_id} 速度:{speed:.2f}m/s 方向:[{velocity[0]:.2f},{velocity[1]:.2f}]")
            else:
                # 如果历史位置不足，则使用当前位置作为预测位置
                predicted_positions.append(cluster['center'])
        
        return predicted_positions

    def motion_model(self, state, u):
        """全向轮运动模型
        state: [x, y, vx, vy]
        u: [ax, ay] 加速度
        """
        new_state = np.zeros_like(state)
        
        # 更新速度
        new_state[2] = state[2] + u[0]*self.dt  # vx
        new_state[3] = state[3] + u[1]*self.dt  # vy
        
        # 限制最大速度
        speed = np.sqrt(new_state[2]**2 + new_state[3]**2)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            new_state[2] *= scale
            new_state[3] *= scale
        
        # 更新位置 - 全向轮直接按机器人坐标系运动
        new_state[0] = state[0] + new_state[2]*self.dt
        new_state[1] = state[1] + new_state[3]*self.dt
        
        return new_state

    def mpc_optimize(self):
        """MPC优化"""
        # 如果没有手动控制且没有目标点，返回None
        if self.goal is None and np.sum(np.abs(self.manual_cmd)) < 0.01:
            return None
            
        # 构建初始状态 [x, y, vx, vy]
        state = np.zeros(4)
        state[:2] = self.pose[:2]
        state[2:4] = self.velocity
        
        # 约束条件 - 限制加速度
        bounds = [(-self.max_accel, self.max_accel)] * 2 * self.prediction_horizon
        
        # 初始控制序列 - 使用当前速度方向
        u0 = np.zeros(2 * self.prediction_horizon)
        if np.linalg.norm(self.velocity) > 0.01:
            vel_direction = self.velocity / np.linalg.norm(self.velocity)
            for i in range(self.prediction_horizon):
                u0[i*2:i*2+2] = vel_direction * self.max_accel * 0.5
        
        try:
            # 确定目标点 - 如果没有目标点但有手动控制输入，创建临时目标点
            temp_goal_created = False
            goal = self.goal
            
            if self.goal is None and np.sum(np.abs(self.manual_cmd)) > 0.01:
                # 创建临时目标点
                temp_goal_created = True
                direction = np.array([self.manual_cmd[0], self.manual_cmd[1]])
                if np.linalg.norm(direction) > 0.01:
                    direction = direction / np.linalg.norm(direction)
                    goal = self.pose[:2] + direction * 3.0
                    self.get_logger().info(f"创建临时目标点: [{goal[0]:.2f}, {goal[1]:.2f}]")
            
            # 获取障碍物预测位置
            obstacle_positions = self.predict_obstacle_positions()
            
            # 只有当有目标点和障碍物时才执行MPC
            if goal is not None:
                # 执行优化
                if len(obstacle_positions) > 0:
                    self.get_logger().info(f"MPC开始优化，障碍物数量: {len(obstacle_positions)}")
                
                res = minimize(
                    lambda u: self.mpc_cost(u, state, goal, obstacle_positions),
                    u0,
                    bounds=bounds,
                    method='SLSQP',
                    options={'maxiter': 100}
                )
                
                if temp_goal_created:
                    goal = None
                
                if res.success:
                    # 提取最优控制序列
                    optimal_controls = res.x
                    
                    # 从第一个控制量计算速度命令
                    cmd_vx = self.velocity[0] + optimal_controls[0] * self.dt
                    cmd_vy = self.velocity[1] + optimal_controls[1] * self.dt
                    
                    # 速度限制
                    speed = np.sqrt(cmd_vx**2 + cmd_vy**2)
                    if speed > self.max_speed:
                        scale = self.max_speed / speed
                        cmd_vx *= scale
                        cmd_vy *= scale
                    
                    # 生成MPC预测轨迹
                    self.mpc_trajectory = []
                    pred_state = state.copy()
                    for i in range(self.prediction_horizon):
                        u_i = [optimal_controls[i*2], optimal_controls[i*2+1]]
                        pred_state = self.motion_model(pred_state, u_i)
                        self.mpc_trajectory.append((pred_state[0], pred_state[1]))
                    
                    # 发布MPC轨迹可视化
                    self.publish_mpc_trajectory(self.mpc_trajectory)
                    
                    # 记录MPC的结果和障碍物信息
                    if len(obstacle_positions) > 0:
                        self.get_logger().info(f"MPC生成避障路径: [{cmd_vx:.2f}, {cmd_vy:.2f}], 障碍物数量: {len(obstacle_positions)}")
                        
                    return np.array([cmd_vx, cmd_vy])
                else:
                    self.get_logger().warn(f"MPC优化失败: {res.message}")
                    # 简单避障策略：远离最近的障碍物
                    if len(self.obstacle_clusters) > 0:
                        # 找到最近的障碍物
                        nearest = min(self.obstacle_clusters, 
                                     key=lambda x: np.linalg.norm(self.pose[:2] - x['center']))
                        away_dir = self.pose[:2] - nearest['center']
                        dist = np.linalg.norm(away_dir)
                        
                        if dist > 0:
                            away_dir = away_dir / dist
                            # 距离越近，避障速度越大
                            avoid_speed = min(self.max_speed, 
                                             self.max_speed * (1.5 / max(0.1, dist)))
                            self.get_logger().warn(f"执行紧急避障! 距离:{dist:.2f}m, 速度:{avoid_speed:.2f}")
                            return away_dir * avoid_speed
                    return None
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"MPC计算错误: {str(e)}")
            return None

    def mpc_cost(self, u, state, goal, obstacles):
        """MPC代价函数 - 调整避障距离"""
        cost = 0.0
        current_state = state.copy()
        horizon = len(u) // 2
        
        for i in range(horizon):
            # 应用控制量
            accel = np.array([u[i*2], u[i*2+1]])
            current_state = self.motion_model(current_state, accel)
            
            # 目标点代价
            if goal is not None:
                goal_weight = self.weight_goal / (i + 1)  # 递减权重
                goal_dist = np.linalg.norm(current_state[:2] - goal)
                cost += goal_weight * goal_dist
            
            # 障碍物代价 - 只在接近时才计算
            safe_distance = self.obstacle_radius + 0.5  # 0.5是车辆半径
            for obs in obstacles:
                dist = np.linalg.norm(current_state[:2] - obs)
                # 修改：使用参数化的避障距离系数
                if dist < safe_distance * self.obstacle_avoid_factor:  
                    # 使用二次惩罚而不是指数函数，避免远距离过度避障
                    obstacle_cost = self.weight_obstacle * ((safe_distance * self.obstacle_avoid_factor) - dist)**2
                    cost += obstacle_cost
            
            # 控制量平滑度代价
            control_cost = self.weight_accel * np.sum(accel**2)
            cost += control_cost
            
            # 惩罚低速状态
            speed = np.sqrt(current_state[2]**2 + current_state[3]**2)
            if speed < 0.1:
                cost += 5.0  # 鼓励运动而不是停止
        
        return cost

    def fuse_commands(self, auto_cmd):
        """融合手动和自动控制指令（调整避障距离）"""
        if auto_cmd is None:
            return self.manual_cmd
        
        # 速度限幅
        manual_vx = np.clip(self.manual_cmd[0], -self.max_speed, self.max_speed)
        manual_vy = np.clip(self.manual_cmd[1], -self.max_speed, self.max_speed)
        
        # 动态调整权重
        dynamic_weight = self.manual_weight
        min_dist = float('inf')
        
        # 只有当有障碍物时才动态调整权重
        if len(self.obstacle_clusters) > 0:
            # 找到最近的障碍物
            for obs in self.obstacle_clusters:
                dist = np.linalg.norm(self.pose[:2] - obs['center'])
                min_dist = min(min_dist, dist)
            
            # 修改：仅在更近距离开始减少手动控制权重
            safe_distance = self.obstacle_radius + 0.5  # 0.5是车辆半径
            if min_dist < 1.5 * safe_distance:  # 修改：从3.0->1.5倍安全距离
                # 修改：使用线性减少而不是二次方
                factor = min(1.0, min_dist / (1.5 * safe_distance))
                dynamic_weight = max(0.2, self.manual_weight * factor)  # 修改：最小权重从0.05->0.2
                self.get_logger().warn(f"接近障碍物，手动控制权重降低至: {dynamic_weight:.3f}")
        
        # 加权融合
        fused_vx = (dynamic_weight * manual_vx + (1-dynamic_weight) * auto_cmd[0])
        fused_vy = (dynamic_weight * manual_vy + (1-dynamic_weight) * auto_cmd[1])
        
        # 记录融合过程，增强可见性
        self.get_logger().info(f"融合: 手动[{manual_vx:.2f},{manual_vy:.2f}]*{dynamic_weight:.2f} + "
                             f"避障[{auto_cmd[0]:.2f},{auto_cmd[1]:.2f}]*{1-dynamic_weight:.2f} = "
                             f"结果[{fused_vx:.2f},{fused_vy:.2f}]")
        
        return np.array([fused_vx, fused_vy])

    def control_cycle(self):
        """主控制循环"""
        try:
            # 没有目标点时，可以使用手动控制或悬停
            if self.goal is None and np.sum(np.abs(self.manual_cmd)) < 0.01:
                # 当没有目标点且无手动控制时，保持静止
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                self.cmd_pub.publish(cmd)
                return
            
            # MPC优化 - 如果启用自动控制
            auto_cmd = None
            if self.manual_weight < 1.0:
                auto_cmd = self.mpc_optimize()
                
            # 指令融合
            fused_cmd = self.fuse_commands(auto_cmd)
            
            # 记录运行状态
            if self.goal is not None:
                goal_dist = np.linalg.norm(self.pose[:2] - self.goal[:2])
                self.get_logger().debug(f"距离目标: {goal_dist:.2f}m, 速度: [{fused_cmd[0]:.2f}, {fused_cmd[1]:.2f}]")
            
            # 检测碰撞（新增）
            if len(self.obstacle_clusters) > 0:
                nearest = min(self.obstacle_clusters, key=lambda x: np.linalg.norm(self.pose[:2] - x['center']))
                dist = np.linalg.norm(self.pose[:2] - nearest['center'])
                if dist < (0.5 + self.obstacle_radius):  # 0.5是车辆半径
                    self.get_logger().error(f"检测到碰撞! 距离障碍物:{dist:.2f}m")
                    # 碰撞时强制减速
                    scale = 0.2
                    fused_cmd = fused_cmd * scale
            
            # 发布控制指令
            cmd = Twist()
            cmd.linear.x = float(fused_cmd[0])
            cmd.linear.y = float(fused_cmd[1])
            cmd.angular.z = 0.0  # 全向轮可以直接平移，不需要旋转
            
            self.cmd_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f"控制循环错误: {str(e)}")
            # 发送停止命令
            cmd = Twist()
            self.cmd_pub.publish(cmd)

    def goal_callback(self, msg):
        """目标点回调"""
        self.goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        self.get_logger().info(f"收到新目标点: [{self.goal[0]:.2f}, {self.goal[1]:.2f}]")

    def manual_callback(self, msg):
        """手动控制回调"""
        self.manual_cmd = np.array([msg.linear.x, msg.linear.y])

def main(args=None):
    rclpy.init(args=args)
    controller = OmniMPCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
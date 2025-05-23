
import math
import random
from geometry_msgs.msg import PoseStamped 
import rclpy
from rclpy.node import Node
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment
from scipy.optimize import minimize
import time

# 设置字体 (确保系统中有对应的字体，支持英文字符)
# Set font (ensure corresponding font is in the system, supports English ]characters)
# try:
#     # Prioritize common universal fonts that are likely to be present
#     plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans']
#     # Keep this for displaying minus signs correctly
#     # 保持这个设置以正确显示负号
#     plt.rcParams['axes.unicode_minus'] = False
# except Exception as e:
#     print(f"Warning: Could not set preferred font. Error: {e}")
#     print("Using default font.")
#     # Fallback to system default
#     # 回退到系统默认字体
#     plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']


class CarController(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('car_controller')
        # 创建订阅者 
        """节点名称根据具体2D雷达的topic名称来设置"""
        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)#10是队列大小
        self.subscription_position = self.get_subscriptions(
            PoseStamped,
            '/laser_position',
            self.position_callback,
            10
        )
 # 防止未使用的变量警告
    
        # # 小车运动参数
        # # Car movement parameters
        # self.x = 0.0
        # self.y = 0.0
        # # 车辆朝向角 (全向车位置更新不需要，但雷达扫描点的位置依赖于它)
        # # Vehicle orientation angle (not needed for omni-directional position update, but radar scan points depend on it)
        # self.theta = 0.0
        # 速度标量
        # Velocity scalar
        self.velocity = 0.0
        # 全向车的速度可以表示为 (vx, vy)
        # Omnidirectional car velocity can be represented as (vx, vy)
        self.vx = 0.0
        self.vy = 0.0
        # 最大速度
        # Maximum speed
        self.max_speed = 5.0
        # 基础加速度 (用于手动控制，作为期望加速度的参考) - Note: MPC uses max_accel
        # Base acceleration (for manual control, as reference for desired acceleration) - Note: MPC uses max_accel
        self.acceleration = 3.0 # This parameter is not currently used by the MPC logic, which commands acceleration directly
        self.trajectory = []

        # *** 添加阻力参数 ***
        # *** Add Resistance Parameters ***
        # 阻力系数，值越大阻力越大 (与速度成正比，单位: s^-1)
        # Resistance coefficient, higher value means more resistance (proportional to velocity, unit: s^-1)
        self.resistance_coeff = 0.8

        # MPC参数
        # MPC parameters
        # MPC 采样时间 (Note: This is different from actual frame dt)
        # MPC sampling time (Note: This is different from actual frame dt)
        self.dt = 0.10 # MPC planning step time
        # 预测步数
        # Prediction horizon
        self.prediction_horizon = 5
        # 最大加速度 (m/s^2) - This is the maximum *control* acceleration MPC can command
        # Maximum acceleration (m/s^2) - This is the maximum *control* acceleration MPC can command
        self.max_accel = 3.0
        # 目标点权重
        # Goal point weight
        self.weight_goal = 10.0
        # 速度匹配权重（可选，让车倾向于达到一定速度，与手动方向相关）
        # Velocity matching weight (optional, makes the car tend towards a certain speed related to manual direction)
        self.weight_velocity = 5.0 # Weight for velocity tracking in manual mode
        # 加速度（控制量）权重
        # Acceleration (control input) weight
        # Keep weight_accel relatively low compared to velocity or goal weights
        # A very low weight_accel can improve responsiveness but may lead to shaky control
        # 0.1 is a reasonable starting point.
        self.weight_accel = 0.1
        # 障碍物权重
        # Obstacle weight
        self.weight_obstacle = 80.0
        # 边界权重
        # Boundary weight
        self.weight_boundary = 50.0
        # 车辆半径 (用于碰撞和避障)
        # Vehicle radius (for collision and avoidance)
        self.car_radius = 0.5

        # 调整的安全距离参数
        # Adjustable safety distance parameters
        # 障碍物可视化/模型半径 (用于碰撞和避障)
        # Obstacle visualization/model radius (for collision and avoidance)
        self.obstacle_visual_radius = 0.3
        # 边界安全距离
        # Boundary safety distance
        self.boundary_safe_distance = 1
        # MPC obstacle avoidance extra margin
        self.avoidance_margin = 0.2
        # Distance beyond safety_zone to start warning penalty
        self.warning_zone_distance = 1.0

        # 世界大小，正方形边长的一半
        # World size, half of the square side length
        self.world_size = 10

        # 避障/手动控制混合参数
        # Obstacle avoidance / Manual control blending parameters
        # 障安全区 + MPC裕量 (当障碍物距离 <= 此距离时，完全优先避障)
        # Obstacle safety zone + MPC margin (when obstacle distance <= this distance, full priority to avoidance)
        self.blend_safe_dist = self.car_radius + self.obstacle_visual_radius + self.avoidance_margin
        # 离障碍物远于此距离时，完全优先目标/手动控制
        # When farther than this distance from an obstacle, full priority to goal/manual control
        self.manual_override_dist = self.blend_safe_dist + 3.0 # Increased blend range slightly
        # 当前帧的混合比例 (1: 纯避障, 0: 纯目标/手动)
        # Current frame's blend ratio (1: pure avoidance, 0: pure goal/manual)
        self.blend_alpha = 0.0

        # 雷达参数 (角度-距离传感器)
        # Radar parameters (Angle-Distance sensor)
        # 雷达最大探测距离
        # Radar maximum detection distance
        self.radar_max_distance = 15.0
        # 雷达角度分辨率 (度)
        # Radar angle resolution (degrees)
        self.radar_angle_res = 1 # Slightly coarser resolution for speed
        # 雷达距离测量噪声标准差
        # Radar distance measurement noise standard deviation
        self.radar_noise_std = 0.1
        # 存储 (angle_deg, distance) 列表
        # Store (angle_deg, distance) list
        self.raw_scan_data = []

        # 障碍物跟踪和预测参数
        # Obstacle tracking and prediction parameters
        # 存储 {id: int, center: (x, y), age: int} - Currently tracked
        # Stores {id: int, center: (x, y), age: int} - Currently tracked
        self.obstacle_clusters = []
        # 存储 id: [pos1, pos2, ...]
        # Stores id: [pos1, pos2, ...]
        self.obstacle_history = {}
        # Store last 10 positions for velocity estimate
        self.max_history_length = 10
        # Used for tracking association threshold
        self.max_expected_obstacle_speed = 1.0

        # DBSCAN 参数
        # DBSCAN parameters
        # Max distance for neighbors
        self.dbscan_eps = 0.5
        # Min points in a cluster
        self.dbscan_min_samples = 3
        # 定时器，每0.1秒执行一次，执行time_callback函数
        self.timer = self.create_timer(0.1, self.time_callback)

        """发布决策信息,其中的消息类型需要根据需求进行修改"""
        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/mpc_decision',
            10
        )
    def scan_callback(self, msg):
        """2D激光雷达数据回调函数"""
        # 获取激光雷达数据并且进行可视化，使用matplotlib
        # self.get_logger().info('Laser scan received')
        # 计算距离最近的障碍物距离
        laser = np.array(msg.ranges)  
    def position_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.theta = msg.pose.orientation.z

    def convert_polar_to_cartesian(self, angle_deg_relative, distance, car_x, car_y, car_theta_rad):
        """将相对于小车的极坐标点 (角度(度), 距离) 转换为世界坐标系的 (x, y)。"""
        # Converts a polar point (angle_deg, distance) relative to the car to world coordinates (x, y).
        # Use car's orientation to convert relative angle to world angle
        # car_theta_rad = math.radians(car_theta_deg)
        angle_rad_relative = math.radians(angle_deg_relative)

        angle_rad_world = car_theta_rad + angle_rad_relative

        world_x = car_x + distance * math.cos(angle_rad_world)
        world_y = car_y + distance * math.sin(angle_rad_world)
        return (world_x, world_y)


    def cluster_scan_points(self):
        """将原始雷达数据 (角度, 距离) 转换为世界坐标 (x, y) 点，并进行聚类。"""
        # Converts raw radar data (angle, distance) to world coordinates (x, y) points and performs clustering.
        # if not self.raw_scan_data:
        #     # Store points for plotting
        #     self.scan_points_for_plot = np.empty((0, 2))
        #     return []

        # # Convert raw scan data (angle, distance) to world coordinates (x, y)
        # # 将原始雷达数据（角度，距离）转换为世界坐标（x，y）点
        world_scan_points = [
            self.convert_polar_to_cartesian(angle, dist, self.x, self.y, self.theta)
            for angle, dist in self.raw_scan_data
        ]
        # # Store points for plotting
        self.scan_points_for_plot = np.array(world_scan_points)

        # # If not enough points for minimum samples, return empty list of centers
        # if self.scan_points_for_plot.shape[0] < self.dbscan_min_samples:
        #     return []

        # Perform DBSCAN clustering on the world coordinates
        # DBSCAN类的实例，导入所需的参数如 eps（聚类半径）0.5 和 min_samples（最小样本数）为 3
        dbscan = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
        # fit_predict 输出每个点所属的簇的标签
        clusters = dbscan.fit_predict(self.scan_points_for_plot)
        # 将每个点所属的簇的标签作为键，将该点作为值，存储在字典中 c
        clustered_points_map = {}
        # 遍历每个点，如果该点所属的簇的标签不是 -1（-1 表示噪声点）
        # 则将所有同一个标签的点存储在同一个键对应的列表中
        for i, label in enumerate(clusters):
            if label != -1: # Ignore noise points (-1)
                if label not in clustered_points_map:
                    clustered_points_map[label] = []
                # 将属于同一个簇的点存储在同一个键对应的列表中
                clustered_points_map[label].append(self.scan_points_for_plot[i])

        obstacle_centers = []
        # 获取每个簇的中心点，将其取平均之后作为该簇（障碍物）的位置
        for label, points in clustered_points_map.items():
            center = np.mean(points, axis=0)
            obstacle_centers.append(center)

        return obstacle_centers


    def track_obstacles(self, current_obstacle_centers, dt):
        """跟踪障碍物，并更新历史位置。使用匈牙利算法匹配当前检测到的中心与已知障碍物。"""
        # Tracks obstacles and updates historical positions. Uses the Hungarian algorithm to match current detected centers with known obstacles.
        # 获取已知障碍物的数量
        num_tracked = len(self.obstacle_clusters)
        # 获取当前检测到的障碍物的数量
        num_detected = len(current_obstacle_centers)

        # Build cost matrix for matching: Distance between tracked centers and detected centers
        #np.full 生成一个 num_tracked * num_detected 的矩阵，每次初始化的时候矩阵的值为无穷大，
        # 用于存储已知障碍物与当前检测到的障碍物之间的距离，
        cost_matrix = np.full((num_tracked, num_detected), np.inf)
        # 计算一个障碍物在 dt 时间内可能移动的最大距离
        # Threshold calculation: Max distance an object *could* move between frames + margin
        # Use actual frame dt for tracking threshold
        # 用最大可能的障碍物速度乘以 dt 得到一个障碍物在 dt 时间内可能移动的最大距离
        max_possible_movement = self.max_expected_obstacle_speed * dt
        # Add margin for robustness due to tracking imperfections, clustering noise, and potential small jitters
        # 添加一个缓冲区，用于考虑跟踪误差、聚类噪声以及潜在的小抖动
        match_threshold = max_possible_movement + self.dbscan_eps * 1.5 # Example margin based on DBSCAN epsilon


        for i in range(num_tracked):
            for j in range(num_detected):
                # linalg.norm 计算两个点之间的欧几里得距离
                dist = np.linalg.norm(np.array(self.obstacle_clusters[i]['center']) - np.array(current_obstacle_centers[j]))
                # Only consider potential matches within the threshold distance
                # 如果两个障碍物之间的距离小于匹配阈值，则认为这两个障碍物是同一个障碍物
                if dist < match_threshold:
                    #既然是同一个障碍物，那么代价矩阵的值就为两个障碍物之间的距离
                    cost_matrix[i, j] = dist # Use distance as cost

        updated_clusters = []
        matched_detected_indices = set() # Indices of detected centers that found a match
        matched_tracked_indices = set() # Indices of tracked clusters that found a match

        if num_tracked > 0 and num_detected > 0:#如果已知障碍物和当前检测到的障碍物都存在
            try:
                # Solve the assignment problem (Hungarian algorithm)
                # 使用匈牙利算法解决分配问题
                # linear_sum_assignment 返回一个行索引列表和列索引列表，
                row_ind, col_ind = linear_sum_assignment(cost_matrix)

                # Process matched pairs
                # 遍历已知障碍物和当前检测到的障碍物的匹配结果
                for i, j in zip(row_ind, col_ind):
                    # Check if the match is valid (cost < infinity, meaning it was within threshold)
                    # 如果两个障碍物之间的距离小于匹配阈值，则认为这两个障碍物是同一个障碍物
                    if cost_matrix[i, j] < np.inf:
                        # 获取已知障碍物，数据类型为字典
                        tracked_cluster = self.obstacle_clusters[i]
                        # 获取当前检测到的障碍物，数据类型为列表
                        detected_center = current_obstacle_centers[j]
                        # Update tracked cluster's center to the new detected position and reset age
                        # 更新已知障碍物的中心点为当前检测到的障碍物的中心点，并重置年龄
                        tracked_cluster['center'] = detected_center
                        tracked_cluster['age'] = 0

                        obstacle_id = tracked_cluster['id']
                        # Update history for velocity estimation
                        if obstacle_id not in self.obstacle_history:
                            # 如果已知障碍物不在历史记录中，则创建一个空列表
                            self.obstacle_history[obstacle_id] = []
                        # 将当前检测到的障碍物的中心点添加到已知障碍物的历史记录中
                        self.obstacle_history[obstacle_id].append(detected_center)
                        #进行实时数据更新，获取新数据的同时，删除旧数据
                        # Keep history length limited
                        if len(self.obstacle_history[obstacle_id]) > self.max_history_length:
                            self.obstacle_history[obstacle_id].pop(0) # Remove the oldest entry
                        # 将已知障碍物添加到更新列表中
                        updated_clusters.append(tracked_cluster)
                        # 将已知障碍物的索引添加到已知障碍物的索引列表中
                        matched_tracked_indices.add(i)
                        # 将当前检测到的障碍物的索引添加到当前检测到的障碍物的索引列表中
                        matched_detected_indices.add(j)

            except ValueError as e:
                # This can occur if the cost matrix contains no finite values (e.g., all distances > match_threshold)
                # or if the matrix dimensions are invalid.
                # print(f"Warning: linear_sum_assignment failed: {e}. Processing unmatched clusters/detections directly.")
                pass # Handle unmatched processing below

        # 处理新障碍物（检测到的但未匹配到任何已知障碍物的障碍物）
        # Handle new obstacles (detected but not matched to any tracked cluster)
        for j in range(num_detected):
            if j not in matched_detected_indices:
                new_center = current_obstacle_centers[j]
                # Assign a new unique ID
                existing_ids = list(self.obstacle_history.keys()) # Look through all history keys, even for aged-out ones, to avoid ID reuse
                new_id = max(existing_ids + [-1]) + 1 if existing_ids else 0
                new_cluster = {'id': new_id, 'center': new_center, 'age': 0}
                updated_clusters.append(new_cluster)
                # Start history for the new obstacle
                self.obstacle_history[new_id] = [new_center]


        # Handle lost obstacles (tracked but not matched to any detected center)
        # Age the tracked clusters that were not matched
        for i in range(num_tracked):
            if i not in matched_tracked_indices:
                lost_cluster = self.obstacle_clusters[i]
                lost_cluster.setdefault('age', 0) # Ensure 'age' key exists
                lost_cluster['age'] += 1
                # Keep the cluster for a few frames even if not detected, using its last known position
                if lost_cluster['age'] < 5: # Age threshold (e.g., 5 frames)
                    # Append the aged cluster with its last known position
                    updated_clusters.append(lost_cluster)
                else:
                    # If too old, remove from active tracking and history
                    if lost_cluster['id'] in self.obstacle_history:
                        del self.obstacle_history[lost_cluster['id']] # Remove history for this ID

        # Update the list of currently tracked obstacles
        self.obstacle_clusters = updated_clusters

    def calculate_blend_ratio(self, dist_to_nearest_obstacle):
        """Calculates blend ratio (alpha) between avoidance (1) and manual (0)."""
        # If no obstacles are tracked or the nearest is far away, pure manual/goal control (alpha=0)
        if dist_to_nearest_obstacle is None or dist_to_nearest_obstacle >= self.manual_override_dist:
            return 0.0
        # If an obstacle is very close, pure avoidance control (alpha=1)
        if dist_to_nearest_obstacle <= self.blend_safe_dist:
            return 1.0

        # In the transition zone, blend linearly based on the distance
        range_dist = self.manual_override_dist - self.blend_safe_dist
        if range_dist <= 1e-9: # Avoid division by zero if the range is tiny
            return 1.0 if dist_to_nearest_obstacle <= self.blend_safe_dist else 0.0

        # Clamp distance within the blending range for calculation
        clamped_dist = np.clip(dist_to_nearest_obstacle, self.blend_safe_dist, self.manual_override_dist)

        # Calculate alpha: 1 at blend_safe_dist, 0 at manual_override_dist
        # This formula maps [blend_safe_dist, manual_override_dist] to [1, 0]
        alpha = (self.manual_override_dist - clamped_dist) / range_dist

        # Ensure alpha is strictly between 0 and 1 (although clip should handle this)
        return np.clip(alpha, 0.0, 1.0)


    def mpc_cost(self, u_flat, x0, goal, predicted_obstacle_paths):
        """MPC 优化目标函数。u_flat是展平的控制输入 [ax0, ay0, ax1, ay1, ...]."""
        # MPC objective function. u_flat is the flattened control input [ax0, ay0, ax1, ay1, ...].
        # u_flat: Control sequence [ax0, ay0, ax1, ay1, ...] from the optimizer (represents *control* acceleration, not net)
        # x0: initial state [x, y, vx, vy]
        # goal: target position [gx, gy] (numpy array or None)
        # predicted_obstacle_paths: list of paths, where each path is a list of predicted positions for each MPC step:
        #   [[pos_obs0_t0, pos_obs0_t1, ..., pos_obs0_tN], [pos_obs1_t0, pos_obs1_t1, ..., pos_obs1_tN], ...]
        # blend_alpha: Current blend ratio (1=avoidance, 0=manual/goal) - Accessed via self.blend_alpha

        cost = 0.0
        state = np.array(x0) # Current state at the start of the horizon

        # Store MPC predicted path for visualization
        # The trajectory starts from the current car position (x0[:2])
        self.mpc_trajectory = [state[:2].tolist()]

        alpha = self.blend_alpha # Get the current blend ratio

        # Scale weights based on the blending ratio
        # Obstacle cost is prioritized when alpha is high
        obstacle_cost_weight_scaled = alpha * self.weight_obstacle
        # Goal/Manual cost is prioritized when alpha is low
        # Use goal weight if a goal is set, otherwise use velocity weight for manual control
        goal_or_manual_weight_scaled = (1.0 - alpha) * (self.weight_goal if goal is not None else self.weight_velocity)


        # Iterate through each step in the MPC prediction horizon
        for i in range(self.prediction_horizon):
            # Extract *control* acceleration for the current step (i) from the optimizer's flattened input
            # Ensure indices are within the bounds of u_flat
            if (i * 2 + 1) < len(u_flat):
                control_i = np.array([u_flat[i * 2], u_flat[i * 2 + 1]])
            else:
                # Fallback to zero control if u_flat is shorter than expected (shouldn't happen with correct bounds/u0 size)
                control_i = np.array([0.0, 0.0])
                # print(f"Warning: u_flat index out of bounds at step {i}. Using zero control.")


            # Predict the next state of the car using the motion model and the control input for this step
            # The motion model includes the effect of resistance.
            # The state `state` is updated iteratively for each step.
            state = self.motion_model(state, control_i, self.dt)
            x, y, vx, vy = state # Unpack the state for easier access

            # Add the predicted position (x, y) at this step to the trajectory for visualization
            self.mpc_trajectory.append(state[:2].tolist())

            # --- Calculate Cost for this step ---

            # 1. Goal/Manual Control Cost (Weighted by 1-alpha)
            goal_or_manual_cost_step = 0.0
            if goal is not None:
                # If a goal is set, penalize distance to the goal (which is static)
                dist_to_goal = np.linalg.norm(state[:2] - goal)
                goal_or_manual_cost_step = dist_to_goal**2 # Quadratic penalty for distance
            else:
                # If no goal is set, penalize deviation from the desired manual velocity
                desired_velocity = self.control_vector * self.max_speed # Desired velocity vector
                velocity_error_sq = np.sum((state[2:] - desired_velocity)**2) # Squared error in velocity vector components
                goal_or_manual_cost_step = velocity_error_sq # Quadratic penalty for velocity error

            # Add the scaled goal/manual cost for this step to the total cost
            cost += goal_or_manual_weight_scaled * goal_or_manual_cost_step


            # 2. Control Effort Cost (Standard weight)
            # Penalize the magnitude of the *control* acceleration commanded by the optimizer
            cost += self.weight_accel * np.sum(control_i**2) # Quadratic penalty on control effort


            # 3. Obstacle Avoidance Cost (Weighted by alpha)
            obstacle_cost_step = 0.0
            # Only calculate obstacle cost if the scaled weight is significant and there are obstacle paths
            if obstacle_cost_weight_scaled > 1e-9 and predicted_obstacle_paths:
                # Define safe and warning distances for obstacles
                safe_distance = self.car_radius + self.obstacle_visual_radius + self.avoidance_margin
                warning_zone_dist_start = safe_distance # Warning starts right outside the safe zone
                warning_zone_dist_end = safe_distance + self.warning_zone_distance # Warning ends at this distance


                # Iterate through each obstacle's predicted path
                # predicted_obstacle_paths[j][i] is the predicted position of obstacle j at time t + i*dt
                for obs_path in predicted_obstacle_paths:
                    # Ensure the obstacle path has a predicted position for this specific step 'i'
                    if len(obs_path) > i:
                        obs_pos_at_i = np.array(obs_path[i]) # Get obstacle position at time t + i*dt

                        # Calculate the distance between the car's predicted position and the obstacle's predicted position at time t + i*dt
                        dist_to_obstacle = np.linalg.norm(state[:2] - obs_pos_at_i)

                        if dist_to_obstacle < safe_distance:
                            # High quadratic penalty for being inside or penetrating the safe distance (collision)
                            penetration = safe_distance - dist_to_obstacle
                            obstacle_cost_step += penetration**2 * 100 # Use a higher multiplier for collision

                        elif dist_to_obstacle < warning_zone_dist_end:
                            # Exponential penalty for being within the warning zone
                            # Penalty is high near safe_distance and approaches 0 near warning_zone_dist_end
                            # Penalty should be 0 when dist >= warning_zone_dist_end
                            # Let's use a sigmoid or related function, or a carefully constructed exponential
                            # A simple inverse distance penalty can also work: 1 / (dist - safe_distance) - 1 / (warning_zone_dist_end - safe_distance) when in range
                            # Or, exp((safe_distance - dist)/k)
                            # Let's refine the exponential penalty: exp(-(dist - safe_distance)/smoothing)
                            # At dist = safe_distance, exp(0) = 1
                            # At dist = warning_zone_dist_end = safe_distance + warning_zone_distance, exp(-warning_zone_distance/smoothing)
                            # We want penalty to be significant near safe_distance and negligible near warning_zone_dist_end.
                            # Use inverse exponential: penalty = exp((safe_distance - dist)/smoothing)
                            smoothing_factor = self.warning_zone_distance / 3.0 # Adjust smoothing based on warning zone width
                            obstacle_cost_step += math.exp((safe_distance - dist_to_obstacle) / smoothing_factor)
                            # An offset might be needed if we want it strictly 0 at warning_zone_dist_end, but a small non-zero value is often acceptable.


            # Add the scaled obstacle cost for this step to the total cost
            cost += obstacle_cost_weight_scaled * obstacle_cost_step


            # 4. Boundary Avoidance Cost (Standard weight)
            # Penalize getting too close to the world boundaries
            boundary_cost_step = 0.0
            # Calculate distance to each boundary
            dist_left = state[0] - (-self.world_size)
            dist_right = self.world_size - state[0]
            dist_bottom = state[1] - (-self.world_size)
            dist_top = self.world_size - state[1]

            boundary_safe_distance = self.boundary_safe_distance

            # Quadratic penalty if distance to a boundary is less than the safe distance
            if dist_left < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_left)**2
            if dist_right < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_right)**2
            if dist_bottom < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_bottom)**2
            if dist_top < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_top)**2

            # Add the boundary cost for this step to the total cost
            cost += self.weight_boundary * boundary_cost_step

        # The self.mpc_trajectory list is populated during the loop iterations

        return cost


    def predict_obstacle_positions(self):
        """根据历史位置预测障碍物在MPC预测步数内的路径。返回 list of [pos_t0, pos_t1, ..., pos_tN] for each obstacle."""
        # Predicts obstacle paths over the MPC prediction horizon based on historical positions. Returns list of paths.
        # Each path is a list of positions: [pos_at_t, pos_at_t+dt, pos_at_t+2*dt, ..., pos_at_t+N*dt]
        predicted_obstacle_paths = []

        for cluster in self.obstacle_clusters:
            obstacle_id = cluster['id']
            current_pos = np.array(cluster['center'])
            # Start the predicted path with the obstacle's current tracked position
            predicted_path = [current_pos.tolist()] # Position at time t (current time)

            # Default prediction is stationary (zero velocity)
            velocity_estimate = np.array([0.0, 0.0])

            # Estimate velocity if enough history points are available (at least 2 points)
            if obstacle_id in self.obstacle_history and len(self.obstacle_history[obstacle_id]) >= 2:
                history = self.obstacle_history[obstacle_id]
                # Use the two most recent history points to calculate a velocity vector
                pos_latest = np.array(history[-1])
                pos_previous = np.array(history[-2])

                # Time difference between the last two history points.
                # Using MPC dt here assumes tracking updates happen roughly aligned with MPC steps,
                # or that MPC dt is used as the time unit for velocity scaling.
                history_time_diff_approx = self.dt # Assume history points are approx MPC dt apart

                if history_time_diff_approx > 1e-9: # Avoid division by near zero
                    velocity_estimate = (pos_latest - pos_previous) / history_time_diff_approx

                # Cap the estimated speed to a maximum value to prevent unrealistic predictions from noise or tracking errors
                current_est_speed = np.linalg.norm(velocity_estimate)
                if current_est_speed > self.max_expected_obstacle_speed:
                    if current_est_speed > 1e-9: # Avoid division by zero
                        velocity_estimate = (velocity_estimate / current_est_speed) * self.max_expected_obstacle_speed
                    else:
                        velocity_estimate = np.array([0.0, 0.0]) # Speed is zero

            # Predict positions for each future time step in the MPC horizon (from step 1 to N)
            current_predicted_pos = current_pos.copy() # Prediction starts from current position
            for i in range(self.prediction_horizon):
                # Predict the position at the next time step (t + (i+1)*dt) assuming constant velocity
                current_predicted_pos = current_predicted_pos + velocity_estimate * self.dt
                predicted_path.append(current_predicted_pos.tolist()) # Add position at t + (i+1)*dt

            predicted_obstacle_paths.append(predicted_path)

        return predicted_obstacle_paths


    def solve_mpc(self, x0, goal, predicted_obstacle_paths):
        """求解 MPC 控制序列。"""
        # Solves for the MPC control sequence.
        # x0: Current state [x, y, vx, vy]
        # goal: Target position [gx, gy] or None
        # predicted_obstacle_paths: List of predicted paths for obstacles over the MPC horizon

        effective_goal = np.array(goal) if goal is not None else None
        # Initial guess for control inputs (zero acceleration for all steps)
        u0 = np.zeros(self.prediction_horizon * 2) # 2 control inputs (ax, ay) per step
        # Bounds for control inputs (limit acceleration magnitude for each axis, for each step)
        bounds = [(-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel)] * self.prediction_horizon


        # Define the objective function that the optimizer will minimize
        # It takes the flattened control sequence u_flat and uses the other parameters (x0, goal, predicted_obstacle_paths)
        # which are available from the CarController object instance.
        objective = lambda u_flat: self.mpc_cost(u_flat, x0, effective_goal, predicted_obstacle_paths)

        # Use scipy.optimize.minimize with the SLSQP method
        # SLSQP supports bounds and is suitable for non-linear problems with constraints (though we only have bounds here)
        result = minimize(objective, u0,
                          method='SLSQP',
                          bounds=bounds,
                          options={'maxiter': 200, 'ftol': 1e-2, 'disp': False}) # *** Using Increased maxiter ***

        # The mpc_cost function (called by minimize) internally calculates and stores
        # the predicted trajectory (self.mpc_trajectory) for the *final* set of control inputs found by the optimizer.

        if result.success:
            # If optimization succeeded, return the optimal control sequence
            return result.x
        else:
            # If optimization failed, print a warning (optional) and return zero control inputs.
            # Returning zeros causes the vehicle to slow down due to resistance, effectively stopping it.
            # print(f"MPC Optimization failed: {result.message}") # Uncomment for debugging
            # Clear the MPC trajectory visualization as the planned path is not valid
            self.mpc_trajectory = []
            return np.zeros(self.prediction_horizon * 2) # Return zero control for all steps


    def update_movement(self, dt):
        """使用MPC求解得到的控制量更新车辆状态。"""
        # Updates vehicle state using the control command obtained from the MPC solver.
        # dt: actual time passed since the last frame (variable)

        # Get the current state of the vehicle
        current_state = np.array([self.x, self.y, self.vx, self.vy])

        # Predict the paths of obstacles over the MPC horizon based on current tracking data
        predicted_obstacle_paths = self.predict_obstacle_positions()

        # Solve the MPC problem to get the optimal control sequence for the current state and predicted obstacles
        # solve_mpc also updates self.mpc_trajectory internally
        u_flat = self.solve_mpc(current_state, self.goal, predicted_obstacle_paths)

        # Extract the *first* control command [ax0, ay0] from the optimized sequence
        # This is the commanded acceleration to apply for the duration of the actual frame dt.
        commanded_ax = 0.0
        commanded_ay = 0.0

        if u_flat is not None and len(u_flat) >= 2:
            commanded_ax = u_flat[0]
            commanded_ay = u_flat[1]
        # If u_flat is None or too short (e.g., solve_mpc failed and returned zeros), commanded_ax/ay remain 0.0

        # Calculate the resistance acceleration based on the *current* velocity
        resistance_ax = -self.resistance_coeff * self.vx
        resistance_ay = -self.resistance_coeff * self.vy

        # Calculate the net acceleration applied to the vehicle in this frame
        # This is the sum of the commanded acceleration from MPC and the resistance acceleration
        net_ax = commanded_ax + resistance_ax
        net_ay = commanded_ay + resistance_ay

        # Update velocity based on the net acceleration and the actual frame time step dt
        self.vx += net_ax * dt
        self.vy += net_ay * dt

        # Apply the speed limit *after* updating the velocity
        current_speed = math.hypot(self.vx, self.vy)
        if current_speed > self.max_speed:
            # Scale the velocity vector back down to the maximum speed magnitude
            if current_speed > 1e-12: # Avoid division by near zero
                scale = self.max_speed / current_speed
                self.vx *= scale
                self.vy *= scale
            else: # Speed is zero or near zero, set to zero explicitly to prevent NaNs
                self.vx = 0.0
                self.vy = 0.0

        # Update the scalar velocity display variable
        self.velocity = math.hypot(self.vx, self.vy)

        # Update position based on the new velocity and the actual frame time step dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Add the current position to the trajectory history for visualization
        self.trajectory.append((self.x, self.y))
        # Limit the length of the trajectory displayed
        max_trajectory_length = 500
        if len(self.trajectory) > max_trajectory_length:
            self.trajectory.pop(0) # Remove the oldest position


    def time_callback(self):
        self.obstacle_centers = self.cluster_scan_points()
        self.track_obstacles(self.obstacle_centers, self.dt)
        
        # Calculate the distance from the car to the nearest currently *tracked* obstacle
        car_pos_np = np.array([self.x, self.y])
        d_min = float('inf') # Initialize min distance to infinity
        # If there are any currently tracked obstacle clusters
        if self.obstacle_clusters:
            # Calculate the distance from the car's position to the center of each tracked obstacle
            distances = [np.linalg.norm(car_pos_np - np.array(c['center'])) for c in self.obstacle_clusters]
            if distances: # Check if the list of distances is not empty
                d_min = min(distances) 
        self.blend_alpha = self.calculate_blend_ratio(d_min)
        predicted_obstacle_paths = self.predict_obstacle_positions()        u_flat = self.solve_mpc(current_state, self.goal, predicted_obstacle_paths)

        self.update_movement(self.dt)

def main(args=None):
    rclpy.init(args=args)
    controller = CarController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()








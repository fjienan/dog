import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import math
import random
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment
from scipy.optimize import minimize
import time

# 设置字体 (确保系统中有对应的字体，支持英文字符)
# Set font (ensure corresponding font is in the system, supports English ]characters)
try:
    # Prioritize common universal fonts that are likely to be present
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans']
    # Keep this for displaying minus signs correctly
    # 保持这个设置以正确显示负号
    plt.rcParams['axes.unicode_minus'] = False
except Exception as e:
    print(f"Warning: Could not set preferred font. Error: {e}")
    print("Using default font.")
    # Fallback to system default
    # 回退到系统默认字体
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']


class CarController:
    def __init__(self):
        # 小车运动参数
        # Car movement parameters
        self.x = 0.0
        self.y = 0.0
        # 车辆朝向角 (全向车位置更新不需要，但雷达扫描点的位置依赖于它)
        # Vehicle orientation angle (not needed for omni-directional position update, but radar scan points depend on it)
        self.theta = 0.0
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
        self.manual_override_dist = self.blend_safe_dist + 1.0 # Increased blend range slightly
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

        # 初始化界面
        # Initialize interface
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)
        # 添加鼠标点击事件
        # Add mouse click event
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        self.setup_plot()

        # 控制状态
        # Control state
        # 手动控制输入方向矢量
        # Manual control input direction vector
        self.control_vector = np.array([0.0, 0.0])

        # 障碍物系统 (True Obstacles for Simulation)
        # Obstacle system (True Obstacles for Simulation)
        self.obstacles = []
        self.init_obstacles()

        # 图形对象
        # Graphics objects
        self.car_circle = self.ax.add_patch(plt.Circle((self.x, self.y), self.car_radius, color='red', zorder=5))
        self.trajectory_line, = self.ax.plot([], [], 'b-', lw=2, alpha=0.7)
        self.info_text = self.ax.text(0.05, 0.95, '', transform=self.ax.transAxes, va='top', fontsize=10)
        self.scan_scatter = self.ax.scatter(np.empty(0), np.empty(0), c='lime', s=5, marker='.', alpha=0.5)
        self.border_rect = plt.Rectangle((-self.world_size, -self.world_size), 2 * self.world_size, 2 * self.world_size, linewidth=1,
                                         edgecolor='red', linestyle='--', facecolor='none', zorder=1)
        self.ax.add_patch(self.border_rect)
        # 绿色虚线: MPC预测轨迹
        # Green dashed line: MPC predicted trajectory
        self.mpc_trajectory_line, = self.ax.plot([], [], 'g--', lw=1.5, alpha=0.8)

        # 目标点
        # Goal point
        self.goal = None
        # 更新目标点标记
        # Update goal point marker
        self.goal_marker, = self.ax.plot([], [], 'rx', markersize=10, zorder=6)

        # 计时器用于控制更新频率
        # Timer for controlling update frequency
        self._last_time = time.time()
        self._current_frame_dt = 0.0 # To store actual dt for display

        # 使用动画进行更平滑的更新
        # Use animation for smoother updates
        from matplotlib.animation import FuncAnimation
        # blit=False is simpler if artists are not tracked explicitly
        # interval=30ms means target 33 FPS. Lowering interval increases target FPS, but requires update_frame to be fast enough
        self.ani = FuncAnimation(self.fig, self.update_frame, frames=None, interval=30, blit=False, repeat=False) # Adjusted interval to 30ms

        print(f"MPC blend parameters: safe_dist={self.blend_safe_dist:.2f}m, override_dist={self.manual_override_dist:.2f}m")
        print(f"Collision/MPC avoidance minimum distance: {self.car_radius + self.obstacle_visual_radius + self.avoidance_margin:.2f}m")
        print(f"Boundary safe distance: {self.boundary_safe_distance:.2f}m")
        print(f"Resistance coefficient: {self.resistance_coeff}")
        print(f"MPC Prediction Horizon: {self.prediction_horizon} steps ({self.prediction_horizon * self.dt:.2f} seconds)")
        print(f"MPC Optimizer Max Iterations (SLSQP): 200") # *** FIX: Increased maxiter ***


    def setup_plot(self):
        """初始化绘图区域"""
        # Initializes the plotting area
        self.ax.set_title("Omnidirectional Autonomous Car - Radar (Angle-Distance) MPC Obstacle Avoidance & Manual Blend\nWASD: Move  Space: Emergency Stop  Mouse Click: Set Goal\nGreen Dashed Line: MPC Predicted Path  Orange Circle: True Obstacle  Green Dot: Radar Point Cloud", fontsize=10)
        self.ax.set_xlim(-self.world_size, self.world_size)
        self.ax.set_ylim(-self.world_size, self.world_size)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')

    def on_key_press(self, event):
        """键盘按下事件"""
        # Key press event handler
        if not hasattr(self, '_pressed_keys'):
            self._pressed_keys = set()
        self._pressed_keys.add(event.key)
        self.update_control_vector()

    def on_key_release(self, event):
        """键盘释放事件"""
        # Key release event handler
        if hasattr(self, '_pressed_keys') and event.key in self._pressed_keys:
            self._pressed_keys.remove(event.key)
        self.update_control_vector()

    def update_control_vector(self):
        """更新控制方向矢量"""
        # Update control direction vector
        if not hasattr(self, '_pressed_keys'):
            self._pressed_keys = set()

        vec = np.array([0.0, 0.0])
        # Prioritize Space for stopping
        if ' ' in self._pressed_keys or 'space' in self._pressed_keys:
            self.vx = 0.0
            self.vy = 0.0
            self.velocity = 0.0
            # Clear manual control direction
            self.control_vector = np.array([0.0, 0.0])
            # Remove space from set so we don't process other keys simultaneously (optional, but makes space a dedicated stop)
            self._pressed_keys.discard(' ')
            self._pressed_keys.discard('space')
            self.goal = None # Clear goal on emergency stop
            self.goal_marker.set_data([], [])
            return

        if 'w' in self._pressed_keys or 'up' in self._pressed_keys:    vec += [0, 1]
        if 's' in self._pressed_keys or 'down' in self._pressed_keys:  vec += [0, -1]
        if 'a' in self._pressed_keys or 'left' in self._pressed_keys: vec += [-1, 0]
        if 'd' in self._pressed_keys or 'right' in self._pressed_keys: vec += [1, 0]

        # 归一化控制方向
        # Normalize control direction
        if np.linalg.norm(vec) > 0:
            self.control_vector = vec / np.linalg.norm(vec)
        else:
            self.control_vector = vec

    def on_mouse_click(self, event):
        """鼠标点击事件，设置目标点"""
        if event.inaxes == self.ax:
            self.goal = (event.xdata, event.ydata)
            # 更新目标点标记
            self.goal_marker.set_data(self.goal[0], self.goal[1])
            print(f"目标点设置为: {self.goal}")
            
            # 清除手动控制方向
            if hasattr(self, '_pressed_keys'):
                self._pressed_keys = set()
            self.control_vector = np.array([0.0, 0.0])
            
            # 计算到目标点的距离
            current_pos = np.array([self.x, self.y])
            goal_pos = np.array(self.goal)
            dist_to_goal = np.linalg.norm(goal_pos - current_pos)
            
            # 如果距离较远，允许保持当前速度
            # 如果距离较近，则减速
            if dist_to_goal < 2.0:
                # 在近距离时减速
                self.vx *= 0.5
                self.vy *= 0.5
                self.velocity = math.hypot(self.vx, self.vy)


    def init_obstacles(self):
        """初始化6个动态障碍物 (真实的模拟对象)"""
        # Initializes 6 dynamic obstacles (true simulation objects)
        if hasattr(self, '_obstacle_patches'):
            for patch in self._obstacle_patches:
                patch.remove()
        self._obstacle_patches = []

        self.obstacles = []
        obstacle_id_counter = 0

        true_obstacle_radius = 0.4

        for i in range(6):
            speed = random.uniform(0.3, 0.6)

            if i < 3: # Circular motion
                motion_type = 'circle'
                angle = random.uniform(0, 2*math.pi)
                radius = random.uniform(4, 8)
                # Ensure initial position is not too close to car origin (0,0)
                while np.linalg.norm([radius * math.cos(angle), radius * math.sin(angle)]) < 3.0:
                    angle = random.uniform(0, 2*math.pi)
                    radius = random.uniform(4, 8)

                cx = radius * math.cos(angle)
                cy = radius * math.sin(angle)


                obstacle = {
                    'id': obstacle_id_counter,
                    'circle': plt.Circle((cx, cy), true_obstacle_radius, color='darkorange', alpha=0.8, zorder=3),
                    'motion_type': motion_type,
                    # Angular speed (rad/s) - Let's use speed for linear component for simplicity consistent with linear type
                    'speed': speed, # This is intended linear speed (m/s) along the circumference
                    # Current angle
                    'angle': angle,
                    # Radius of circular path
                    'radius': radius,
                    # Store true radius
                    'true_radius': true_obstacle_radius
                }

                self.ax.add_patch(obstacle['circle'])
                self._obstacle_patches.append(obstacle['circle'])

            else: # Linear motion
                motion_type = 'linear'
                cx = random.uniform(-self.world_size + true_obstacle_radius, self.world_size - true_obstacle_radius)
                cy = random.uniform(-self.world_size + true_obstacle_radius, self.world_size - true_obstacle_radius)

                # Ensure initial position is not too close to car origin (0,0)
                while np.linalg.norm([cx, cy]) < 3.0:
                    cx = random.uniform(-self.world_size + true_obstacle_radius, self.world_size - true_obstacle_radius)
                    cy = random.uniform(-self.world_size + true_obstacle_radius, self.world_size - true_obstacle_radius)


                target_x = random.uniform(-self.world_size + true_obstacle_radius, self.world_size - true_obstacle_radius)
                target_y = random.uniform(-self.world_size + true_obstacle_radius, self.world_size - true_obstacle_radius)

                obstacle = {
                    'id': obstacle_id_counter,
                    'circle': plt.Circle((cx, cy), true_obstacle_radius, color='darkorange', alpha=0.8, zorder=3),
                    # Speed of linear movement (m/s)
                    'motion_type': motion_type,
                    'speed': speed,
                    'target_pos': (target_x, target_y),
                    'true_radius': true_obstacle_radius
                }
                self.ax.add_patch(obstacle['circle'])
                self._obstacle_patches.append(obstacle['circle'])

            self.obstacles.append(obstacle)
            obstacle_id_counter += 1

        print(f"Initialized {len(self.obstacles)} simulation obstacles.")


    def ray_intersect_circle(self, origin, theta_rad, circle_center_pos, circle_radius):
        """射线与圆求交。返回交点在射线上的参数 t (距离)，如果无交点或交点在射线反方向则返回None。"""
        # Calculates intersection of a ray with a circle. Returns parameter t (distance) along the ray, or None if no intersection or intersection is behind the ray origin.
        dx, dy = math.cos(theta_rad), math.sin(theta_rad)
        cx, cy = circle_center_pos
        ox, oy = origin

        fx = ox - cx
        fy = oy - cy
        r = circle_radius

        A = dx*dx + dy*dy # Should be 1 if dx, dy are from a unit vector, but calculated this way is robust
        B = 2 * (fx * dx + fy * dy)
        C = fx * fx + fy * fy - r * r

        delta = B * B - 4 * A * C

        if delta < 0:
            return None # No real intersection

        sqrt_delta = math.sqrt(delta)

        # Handle case where ray might start inside the circle.
        # t represents the distance along the ray. We want t >= 0.
        # If the origin is inside, one solution for t will be negative and one positive.
        # We need the smallest NON-NEGATIVE t.
        # (Note: Ray starts AT the origin, so t=0 is the origin itself. Intersection must be strictly beyond origin for a detection.)

        t1 = (-B - sqrt_delta) / (2 * A)
        t2 = (-B + sqrt_delta) / (2 * A)

        # Get the smallest positive t value
        positive_ts = [t for t in [t1, t2] if t > 1e-9] # Use a small epsilon to avoid detecting collision at the origin point

        if not positive_ts:
            return None # No intersection in front of the ray origin

        min_t = min(positive_ts)

        return min_t


    def scan_environment(self):
        """模拟角度-距离雷达扫描环境。返回(角度(度), 距离)的列表。"""
        # Simulates an angle-distance radar scan of the environment. Returns a list of (angle_deg, distance).
        self.raw_scan_data = []
        origin = (self.x, self.y)

        # Radar scans relative to the car's orientation self.theta
        car_orientation_rad = math.radians(self.theta)

        for angle_deg_relative in np.arange(0, 360, self.radar_angle_res):
            # Convert relative angle to world angle based on car's orientation
            angle_rad_world = car_orientation_rad + math.radians(angle_deg_relative)

            closest_dist = self.radar_max_distance
            hit_found = False

            # 寻找这条射线与所有真实障碍物的最近交点距离
            # Find the nearest intersection distance of this ray with all true obstacles
            for obstacle in self.obstacles:
                # Use true radius for detection
                t = self.ray_intersect_circle(origin, angle_rad_world, obstacle['circle'].center, obstacle['true_radius'])

                # Only consider hits within max distance
                if t is not None and t < closest_dist:
                    closest_dist = t
                    hit_found = True

            # If found a hit within max distance, record noisy distance measurement
            if hit_found:
                # Add noise to the measured distance
                noisy_dist = closest_dist + random.gauss(0, self.radar_noise_std)
                # Ensure distance is non-negative and within a reasonable range
                noisy_dist = max(0.0, noisy_dist)
                # Cap noisy distance, but allow slightly beyond max_distance due to noise
                noisy_dist = min(noisy_dist, self.radar_max_distance + self.radar_noise_std * 3) # Allow noise to slightly exceed max range

                # Only add points within the nominal max radar distance for clustering/processing
                # Points slightly beyond max_distance due to noise are filtered here before clustering
                if noisy_dist <= self.radar_max_distance:
                    self.raw_scan_data.append((angle_deg_relative, noisy_dist))


    def convert_polar_to_cartesian(self, angle_deg_relative, distance, car_x, car_y, car_theta_deg):
        """将相对于小车的极坐标点 (角度(度), 距离) 转换为世界坐标系的 (x, y)。"""
        # Converts a polar point (angle_deg, distance) relative to the car to world coordinates (x, y).
        # Use car's orientation to convert relative angle to world angle
        car_theta_rad = math.radians(car_theta_deg)
        angle_rad_relative = math.radians(angle_deg_relative)

        angle_rad_world = car_theta_rad + angle_rad_relative

        world_x = car_x + distance * math.cos(angle_rad_world)
        world_y = car_y + distance * math.sin(angle_rad_world)
        return (world_x, world_y)


    def cluster_scan_points(self):
        """将原始雷达数据 (角度, 距离) 转换为世界坐标 (x, y) 点，并进行聚类。"""
        # Converts raw radar data (angle, distance) to world coordinates (x, y) points and performs clustering.
        if not self.raw_scan_data:
            # Store points for plotting
            self.scan_points_for_plot = np.empty((0, 2))
            return []

        # Convert raw scan data (angle, distance) to world coordinates (x, y)
        # 将原始雷达数据（角度，距离）转换为世界坐标（x，y）点
        world_scan_points = [
            self.convert_polar_to_cartesian(angle, dist, self.x, self.y, self.theta)
            for angle, dist in self.raw_scan_data
            # Filter points again just in case (should be filtered by scan_environment)
            if dist <= self.radar_max_distance
        ]
        # Store points for plotting
        self.scan_points_for_plot = np.array(world_scan_points)

        # If not enough points for minimum samples, return empty list of centers
        if self.scan_points_for_plot.shape[0] < self.dbscan_min_samples:
            return []

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


    def predict_obstacle_positions(self):
        """根据历史位置预测障碍物在MPC预测步数内的路径。返回 list of [pos_t0, pos_t1, ..., pos_tN] for each obstacle."""
        # Predicts obstacle paths over the MPC prediction horizon based on historical positions. Returns list of paths.
        # Each path is a list of positions: [pos_at_t, pos_at_t+dt, ..., pos_at_t+N*dt]
        predicted_obstacle_paths = []

        for cluster in self.obstacle_clusters:
            obstacle_id = cluster['id']
            current_pos = np.array(cluster['center'])
            # Start path with the current detected position of the obstacle
            predicted_path = [current_pos.tolist()]

            # Default velocity is zero (stationary prediction)
            velocity_estimate = np.array([0.0, 0.0])

            # Estimate velocity if enough history exists (at least 2 points)
            if obstacle_id in self.obstacle_history and len(self.obstacle_history[obstacle_id]) >= 2:
                history = self.obstacle_history[obstacle_id]
                # Use the last two points in history to estimate velocity
                pos_latest = np.array(history[-1])
                pos_previous = np.array(history[-2])

                # Approximate time difference between the last two history points.
                # Using MPC dt here simplifies prediction steps later.
                # A more accurate approach would use timestamps if available.
                history_time_diff_approx = self.dt # Assume history points are roughly MPC dt apart

                if history_time_diff_approx > 1e-9: # Avoid division by near-zero
                    velocity_estimate = (pos_latest - pos_previous) / history_time_diff_approx

                # Cap estimated speed to a reasonable maximum to avoid prediction errors from noisy data
                current_est_speed = np.linalg.norm(velocity_estimate)
                if current_est_speed > self.max_expected_obstacle_speed:
                    if current_est_speed > 1e-9:
                        velocity_estimate = (velocity_estimate / current_est_speed) * self.max_expected_obstacle_speed
                    else:
                        velocity_estimate = np.array([0.0, 0.0]) # Speed is zero, capping does nothing

            # Predict positions for each step in the MPC horizon (from step 1 to N)
            current_predicted_pos = current_pos.copy() # Start prediction from current tracked position
            for i in range(self.prediction_horizon):
                # Predict next position assuming constant velocity over the MPC dt step
                current_predicted_pos = current_predicted_pos + velocity_estimate * self.dt
                predicted_path.append(current_predicted_pos.tolist()) # Add position at t + (i+1)*dt

            predicted_obstacle_paths.append(predicted_path)

        return predicted_obstacle_paths


    def motion_model(self, state, control, dt):
        """车辆运动模型：state = [x, y, vx, vy]. control = [ax_control, ay_control]. 包含基于当前速度的阻力项。"""
        x, y, vx, vy = state
        ax_control, ay_control = control

        # 计算到目标点的距离和方向
        if self.goal is not None:
            goal_pos = np.array(self.goal)
            current_pos = np.array([x, y])
            dist_to_goal = np.linalg.norm(goal_pos - current_pos)
            direction_to_goal = (goal_pos - current_pos) / (dist_to_goal + 1e-6)  # 避免除以零
            
            # 根据距离动态调整加速度
            # 当距离大于5米时，使用最大加速度
            # 当距离小于2米时，开始减速
            if dist_to_goal > 5.0:
                accel_scale = 1.0  # 最大加速度
            elif dist_to_goal < 2.0:
                # 在2米内开始减速，距离越近减速越快
                accel_scale = dist_to_goal / 2.0
            else:
                # 在2-5米之间平滑过渡
                accel_scale = 1.0
            
            # 应用加速度缩放
            ax_control *= accel_scale
            ay_control *= accel_scale

            # 确保加速度方向指向目标点
            current_speed = math.hypot(vx, vy)
            if current_speed < self.max_speed:
                # 如果速度未达到最大值，增加加速度
                ax_control += direction_to_goal[0] * self.max_accel * 0.5
                ay_control += direction_to_goal[1] * self.max_accel * 0.5

        # 计算阻力加速度
        ax_resistance = -self.resistance_coeff * vx
        ay_resistance = -self.resistance_coeff * vy

        # 计算净加速度
        net_ax = ax_control + ax_resistance
        net_ay = ay_control + ay_resistance

        # 更新速度
        next_vx = vx + net_ax * dt
        next_vy = vy + net_ay * dt

        # 应用速度限制
        current_speed_sq = next_vx**2 + next_vy**2
        max_speed_sq = self.max_speed**2
        if current_speed_sq > max_speed_sq:
            if current_speed_sq > 1e-12:
                scale = self.max_speed / math.sqrt(current_speed_sq)
                next_vx *= scale
                next_vy *= scale
            else:
                next_vx = 0.0
                next_vy = 0.0

        # 更新位置
        next_x = x + next_vx * dt
        next_y = y + next_vy * dt

        return np.array([next_x, next_y, next_vx, next_vy])


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
        current_state = np.array([self.x, self.y, self.vx, self.vy])

        # 预测障碍物路径
        predicted_obstacle_paths = self.predict_obstacle_positions()

        # 求解MPC问题
        u_flat = self.solve_mpc(current_state, self.goal, predicted_obstacle_paths)

        # 提取第一个控制命令
        commanded_ax = 0.0
        commanded_ay = 0.0

        if u_flat is not None and len(u_flat) >= 2:
            commanded_ax = u_flat[0]
            commanded_ay = u_flat[1]

        # 计算阻力加速度
        resistance_ax = -self.resistance_coeff * self.vx
        resistance_ay = -self.resistance_coeff * self.vy

        # 计算净加速度
        net_ax = commanded_ax + resistance_ax
        net_ay = commanded_ay + resistance_ay

        # 更新速度
        self.vx += net_ax * dt
        self.vy += net_ay * dt

        # 应用速度限制
        current_speed = math.hypot(self.vx, self.vy)
        if current_speed > self.max_speed:
            if current_speed > 1e-12:
                scale = self.max_speed / current_speed
                self.vx *= scale
                self.vy *= scale
            else:
                self.vx = 0.0
                self.vy = 0.0

        # 更新标量速度显示
        self.velocity = math.hypot(self.vx, self.vy)

        # 更新位置
        self.x += self.vx * dt
        self.y += self.vy * dt

        # 更新轨迹历史
        self.trajectory.append((self.x, self.y))
        max_trajectory_length = 500
        if len(self.trajectory) > max_trajectory_length:
            self.trajectory.pop(0)


    def check_collision(self, obstacle_positions):
        """碰撞检测：检查小车是否与提供的障碍物位置列表发生碰撞。"""
        # Collision detection: Checks if the car collides with the provided list of obstacle positions.
        # For simulation realism, we check against the *true* obstacle positions, not the tracked/predicted ones.
        car_pos = np.array([self.x, self.y])
        # The obstacle_positions argument is not used; we iterate through the true obstacles in self.obstacles
        for obstacle in self.obstacles:
            obs_pos = np.array(obstacle['circle'].center)
            # Use the true radius of the obstacle for the collision check
            obs_radius = obstacle['true_radius']
            collision_distance = self.car_radius + obs_radius
            dist = np.linalg.norm(car_pos - obs_pos)
            if dist < collision_distance:
                return True # Collision detected
        return False # No collision detected


    def update_frame(self, frame=None):
        """主更新循环"""
        # Main update loop called by FuncAnimation for each frame

        # --- Timing ---
        current_time = time.time()
        # Calculate the actual time elapsed since the last frame
        dt = max(0.001, current_time - self._last_time) # Ensure dt is not zero
        self._last_time = current_time
        self._current_frame_dt = dt # Store for display

        # Clamp max dt to prevent simulation instability if a frame takes too long
        dt = min(dt, 0.1) # e.g., simulate at most 10 FPS even if rendering is slower


        # --- Simulation Updates (True World) ---
        # Update the positions of the true obstacles in the simulation
        for obstacle in self.obstacles:
            if obstacle['motion_type'] == 'circle':
                cx_old, cy_old = obstacle['circle'].center
                radius = obstacle['radius'] # Fixed radius from initialization

                # If radius is non-zero, calculate angular speed and update angle
                if radius > 1e-9:
                    # Linear speed along circumference = radius * angular_speed (rad/s)
                    # So, angular_speed = linear_speed / radius
                    angular_speed_rad_per_sec = obstacle['speed'] / radius
                    angle_moved = angular_speed_rad_per_sec * dt # Change in angle based on actual frame dt
                    obstacle['angle'] = (obstacle['angle'] + angle_moved) % (2 * math.pi) # Update angle, wrap around 2*pi
                    # Calculate new position based on updated angle and radius
                    cx_new = radius * math.cos(obstacle['angle'])
                    cy_new = radius * math.sin(obstacle['angle'])
                    obstacle['circle'].center = (cx_new, cy_new)
                else:
                    # If radius is zero, the obstacle is fixed at the origin (0,0)
                    obstacle['circle'].center = (0.0, 0.0)


            elif obstacle['motion_type'] == 'linear':
                target = np.array(obstacle['target_pos'])
                current = np.array(obstacle['circle'].center)
                direction = target - current
                dist_to_target = np.linalg.norm(direction)

                movement_this_step = obstacle['speed'] * dt # Distance to move based on actual frame dt

                # If the obstacle has reached or almost reached its target
                if dist_to_target < max(movement_this_step * 0.5, 0.05): # Check if remaining distance is less than half a step or a small threshold
                    # Set position exactly at the target to avoid overshooting
                    cx, cy = target
                    obstacle['circle'].center = (cx, cy)
                    # Pick a new random target position within the world bounds, respecting obstacle radius
                    obs_radius = obstacle['true_radius']
                    new_target_x = random.uniform(-self.world_size + obs_radius, self.world_size - obs_radius)
                    new_target_y = random.uniform(-self.world_size + obs_radius, self.world_size - obs_radius)
                    obstacle['target_pos'] = (new_target_x, new_target_y)
                    # print(f"Obstacle {obstacle['id']} reached target, setting new target: {obstacle['target_pos']}") # Debug print
                else:
                    # If target not reached, move towards the target
                    if dist_to_target > 1e-9: # Avoid division by near zero
                        # Calculate movement vector
                        move_vec = direction / dist_to_target * movement_this_step
                        # Update position
                        cx, cy = current + move_vec
                        obstacle['circle'].center = (cx, cy)
                    # If dist_to_target is zero, current position is already the target, no movement needed this step


        # --- Perception and Tracking ---
        # Simulate the radar scan from the car's current position, detecting true obstacles
        self.scan_environment()
        # Process the raw scan data: convert to world points and cluster them to identify obstacle centers
        current_obstacle_centers_world = self.cluster_scan_points()
        # Track the identified obstacle centers over time, updating obstacle_clusters and obstacle_history
        self.track_obstacles(current_obstacle_centers_world, dt)
        # Prediction of obstacle paths for MPC is done *inside* update_movement

        # --- Blend Calculation ---
        # Calculate the distance from the car to the nearest currently *tracked* obstacle
        car_pos_np = np.array([self.x, self.y])
        d_min = float('inf') # Initialize min distance to infinity
        # If there are any currently tracked obstacle clusters
        if self.obstacle_clusters:
            # Calculate the distance from the car's position to the center of each tracked obstacle
            distances = [np.linalg.norm(car_pos_np - np.array(c['center'])) for c in self.obstacle_clusters]
            if distances: # Check if the list of distances is not empty
                d_min = min(distances) # Find the minimum distance

        # Calculate the blending ratio (alpha) based on the minimum distance to a tracked obstacle
        # alpha=1 (pure avoidance) when close, alpha=0 (pure manual/goal) when far
        self.blend_alpha = self.calculate_blend_ratio(d_min)


        # --- Control and Movement ---
        # Update the car's state (position and velocity) using the MPC controller output
        # The MPC solver is called internally by update_movement
        self.update_movement(dt)


        # --- Collision Check ---
        # Check for collision between the car and the true obstacles *after* the movement update
        # If a collision is detected
        if self.check_collision(None): # The argument is ignored, checks against self.obstacles
            # Stop the vehicle immediately
            self.vx = 0.0
            self.vy = 0.0
            self.velocity = 0.0
            # Update info text to show collision status
            self.info_text.set_text("! COLLISION ! Speed Reset")
            # Clear any active goal, as collision implies goal is likely no longer reachable or relevant
            self.goal = None
            self.goal_marker.set_data([], []) # Hide the goal marker
            print("Collision detected! Vehicle stopped.")
            # No further actions needed here like pausing, the animation loop continues but vehicle is stopped.


        else:
            # If no collision, update the information text displayed on the plot
            info_str = f"Speed: {self.velocity:.1f} m/s (vx:{self.vx:.1f}, vy:{self.vy:.1f})\n" \
                       f"Actual Frame dt: {self._current_frame_dt*1000:.1f} ms\n" \
                       f"Manual Direction: ({self.control_vector[0]:.1f}, {self.control_vector[1]:.1f})\n" \
                       f"Nearest Obstacle (tracked): {d_min:.2f} m, MPC Blend Ratio (Avoidance): {self.blend_alpha:.2f}"

            # Add information about the goal if one is set
            if self.goal:
                car_pos_np = np.array([self.x, self.y])
                goal_pos_np = np.array(self.goal)
                dist_to_goal = np.linalg.norm(car_pos_np - goal_pos_np)
                info_str += f"\nGoal: ({self.goal[0]:.1f}, {self.goal[1]:.1f}) Distance: {dist_to_goal:.1f} m"
                
                # 当距离目标点很近时，逐渐减速而不是突然停止
                if dist_to_goal < 2:  # 接近目标点
                    # 计算减速因子，距离越近减速越快
                    decel_factor = 0.9
                    # 应用减
                    self.vx *= decel_factor
                    self.vy *= decel_factor
                    self.velocity = math.hypot(self.vx, self.vy)
                    
                    # 只有当速度非常小时才清除目标点
                    if self.velocity < 0.1:  # 速度阈值
                        self.goal = None  # 清除目标点
                        self.goal_marker.set_data([], [])  # 隐藏目标点标记
                        info_str += "\nGoal Reached!"
                        print("Goal reached! Vehicle stopped smoothly.")

            # Update the info text display
            self.info_text.set_text(info_str)


        # --- Graphics Updates ---

        # Update the position of the car circle graphic
        self.car_circle.center = (self.x, self.y)

        # Update the scatter plot for radar scan points
        # Ensure scan_points_for_plot is a valid numpy array before updating
        if hasattr(self, 'scan_points_for_plot') and isinstance(self.scan_points_for_plot, np.ndarray) and self.scan_points_for_plot.ndim == 2 and self.scan_points_for_plot.shape[1] == 2:
            self.scan_scatter.set_offsets(self.scan_points_for_plot)
        else:
            # If no points or invalid data, clear the scatter plot data
            self.scan_scatter.set_offsets(np.empty((0, 2)))


        # Update the vehicle trajectory line plot
        if self.trajectory:
            traj_np = np.array(self.trajectory)
            self.trajectory_line.set_data(traj_np[:, 0], traj_np[:, 1])
        else:
            self.trajectory_line.set_data([], []) # Clear line if trajectory is empty


        # Update the MPC predicted trajectory line plot
        # self.mpc_trajectory is updated inside mpc_cost or cleared in solve_mpc on failure
        if hasattr(self, 'mpc_trajectory') and self.mpc_trajectory:
            if len(self.mpc_trajectory) > 0:
                mpc_traj_np = np.array(self.mpc_trajectory)
                self.mpc_trajectory_line.set_data(mpc_traj_np[:, 0], mpc_traj_np[:, 1])
            else:
                self.mpc_trajectory_line.set_data([], []) # Clear line if the list is empty
        else:
            self.mpc_trajectory_line.set_data([], []) # Clear line if the attribute is missing or None


        # The positions of the true obstacle patches are updated directly on the patch objects earlier in the frame.

        # Redraw the canvas - FuncAnimation with blit=False handles this automatically.
        # self.fig.canvas.draw_idle() # Not strictly needed with blit=False, but harmless


    def show(self):
        """显示matplotlib窗口并启动动画"""
        # Displays the matplotlib window and starts the animation
        plt.show()
        # 按下q键退出
        plt.close()

# 运行系统
# Run the system
if __name__ == "__main__":
    controller = CarController()
    controller.show()
import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
# from laser_position.msg._laser_position import LaserPosition
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import threading
from scipy.optimize import fsolve
# from sensor_msgs.msg import Imu
# from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
# from laser_interfaces.msg import LaserPosition
# 设置字体为 SimHei 以支持中文显示
# rcParams['font.sans-serif'] = ['SimHei']
# rcParams['axes.unicode_minus'] = False
#12.47cm
# 模拟参数
LASER_data_flag = 1		# 是否有LASER	数据
# FIELD_SIZE = [15, 8]           # 场地大小
LASER_ANGLES = [0, np.deg2rad(72), np.deg2rad(144), np.deg2rad(216), np.deg2rad(288)]
# self.real.precess_noise_std = [0.1, 0.1, 0.05],
# self.real.measurement_noise_std=[0.5,0.5,0.1], 
# self.real.START_POINT = [7.5, 4]       # 初始位置
# LASER_ALLOWED_NOISE = 2      # 激光测量允许的噪声


class Real(Node):

    class EstRobot:
        def __init__(self, real):
            self.lock = threading.Lock()
            self.real = real
            self.est_yaw = 1.0 # 估计朝向
            self.est_pos = np.array(self.real.START_POINT, dtype=np.float64)  # 估计位置
            self.est_vel_x = 0.0
            self.est_vel_y = 0.0
            # self.acc_body = np.array([self.real.acc_x, self.real.acc_y]).reshape((2, 1))
            self.acc_body = np.zeros((2, 1))  # 初始化为零
            self.acc_global = np.zeros((2, 1))  # 新增必要属性
            self.start_time = 0
            self.pre_angular_rate = 0
            self.angular_acc = 0

        def update_imu(self):
            # print(f"Data before IMU update: pos({self.est_pos[0]:.4f}, {self.est_pos[1]:.4f}), vel({self.est_vel_x:.4f}, {self.est_vel_y:.4f}), yaw({self.est_yaw:.4f})")
            if not hasattr(self, "initial_angle"):
                self.initial_angle = self.real.yaw
            
            # 更新角度
            self.est_yaw = self.real.yaw - self.initial_angle

            # 测零漂
            if not hasattr(self, "imu_update_time"):
                self.imu_update_time = 0
                self.mean_imu_yaw = 0
                self.mean_imu_acc_x = 0
                self.mean_imu_acc_y = 0
            self.mean_imu_yaw = (self.mean_imu_yaw * self.imu_update_time + self.est_yaw) / (self.imu_update_time + 1)
            self.mean_imu_acc_x = (self.mean_imu_acc_x * self.imu_update_time + self.real.acc_x) / (self.imu_update_time + 1)
            self.mean_imu_acc_y = (self.mean_imu_acc_y * self.imu_update_time + self.real.acc_y) / (self.imu_update_time + 1)
            self.imu_update_time += 1
            if self.imu_update_time % 50 == 0:
                # 输出到文件和控制台
                # with open("/home/ares/dog/imu_data.txt", "a") as f:
                #     f.write(f"Mean: IMU Yaw: {self.mean_imu_yaw:.4f}, Acc X: {self.mean_imu_acc_x:.4f}, Acc Y: {self.mean_imu_acc_y:.4f}, Update Time: {self.imu_update_time}\n")
                self.real.get_logger().info(f"Mean: IMU Yaw: {self.mean_imu_yaw:.4f}, Acc X: {self.mean_imu_acc_x:.4f}, Acc Y: {self.mean_imu_acc_y:.4f}, Update Time: {self.imu_update_time}")
            
            # 确保三角函数输入为标量
            cos_yaw = np.cos(self.est_yaw)
            sin_yaw = np.sin(self.est_yaw)
            
            # 正确构造2x2旋转矩阵
            R = np.array([
                [cos_yaw, -sin_yaw],
                [sin_yaw,  cos_yaw]
            ], dtype=np.float64)
            
            # 验证维度
            assert R.shape == (2, 2), f"旋转矩阵维度错误: {R.shape}"

            # 确保加速度数据为2x1列向量
            self.acc_body = np.array([[self.real.acc_x], [self.real.acc_y]], dtype=np.float64)
            
            # 检查矩阵维度
            assert self.acc_body.shape == (2, 1), f"acc_body维度错误: {self.acc_body.shape}"

            # 坐标变换
            # R = np.array([
            # 	[np.cos(self.est_yaw), -np.sin(self.est_yaw)],
            # 	[np.sin(self.est_yaw), np.cos(self.est_yaw)]
            # ], dtype=np.float64)

            assert R.shape == (2, 2), f"旋转矩阵维度错误: {R.shape}"

            self.acc_global = R @ self.acc_body

            if not hasattr(self, "last_time"):
                self.last_time = time.time()
            else:
                current_time = time.time()
                dt = current_time - self.last_time
                # 运动学更新
                self.est_vel_x += self.acc_global[0, 0] * dt
                self.est_vel_y += self.acc_global[1, 0] * dt
                self.est_pos[0] += self.est_vel_x * dt
                self.est_pos[1] += self.est_vel_y * dt
                # 更新最后时间
                self.last_time = current_time

            if self.imu_update_time % 1000 == 0:
                self.real.get_logger().info(f"IMU: pos({self.est_pos[0]:.4f}, {self.est_pos[1]:.4f}), vel({self.est_vel_x:.4f}, {self.est_vel_y:.4f}), yaw: {self.est_yaw:.4f}, last_time: {self.last_time:.4f}")

        def update_laser(self):
            global LASER_data_flag
            laser_data = self.real.real_laser.copy()

            # 过滤无效数据 (NaN/inf)
            # laser_data = np.nan_to_num(laser_data, nan=-1, posinf=self.real.FIELD_SIZE[0], neginf=0.0)
            laser_data = [7.5 - 0.1247,4.206 - 0.1247,6.805 - 0.1247,6.805 - 0.1247,4.206 - 0.1247]
            # laser_data = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            laser_x = []
            laser_y = []
            data = []
            up_width = []
            down_width = []
            flag = [0] * len(LASER_ANGLES)
            theory_length_map=[0] *len(LASER_ANGLES)
            # 计算激光的理论长度
            for i in range(len(LASER_ANGLES)): #计算激光的理论长度
                data.append(laser_data[i] + self.real.DELTA_DISTANCE) #获取每个激光的数据
                if data[i] == -1:
                    flag[i] = 1
                    continue
                laser_yaw = (LASER_ANGLES[i] + self.est_yaw) # % (2 * np.pi)
                d_x = d_y = 1145141919810
                if laser_yaw == 0: 		#当激光打在右边界上时
                    thorey_length = d_x = self.real.FIELD_SIZE[0] - self.est_pos[0]
                elif laser_yaw == np.pi: 		#当激光打在左边界上时
                    thorey_length = d_x = self.est_pos[0]
                elif laser_yaw == np.pi / 2: 		#当激光打在上边界上时
                    thorey_length = d_y = self.real.FIELD_SIZE[1] - self.est_pos[1]
                elif laser_yaw == -np.pi / 2:		 #当激光打在下边界上时
                    thorey_length = d_y = self.est_pos[1]
                else: 		#当激光打在其他地方时
                    d_x = (self.real.FIELD_SIZE[0] - self.est_pos[0]) / np.cos(laser_yaw) if np.cos(laser_yaw) > 0 else -self.est_pos[0] / np.cos(laser_yaw)
                    d_y = (self.real.FIELD_SIZE[1] - self.est_pos[1]) / np.sin(laser_yaw) if np.sin(laser_yaw) > 0 else -self.est_pos[1] / np.sin(laser_yaw)
                    thorey_length = d_x if d_x < d_y else d_y
                if abs(data[i] - thorey_length) <= self.real.LASER_ALLOWED_NOISE and d_x > d_y:
                    if np.sin(laser_yaw) > 0:
                        up_width.append([data[i], i])
                    else:
                        down_width.append([data[i], i])
                elif abs(data[i] - thorey_length) > self.real.LASER_ALLOWED_NOISE: #当激光数据与理论数据差距大于允许的噪声时,认为激光数据无效
                    flag[i] = 1 	#-1代表激光数据无效
                theory_length_map[i] = thorey_length
            
            # self.real.get_logger().info(f"Laser flag: {flag}, data: {data}")
            # self.real.get_logger().info(f"Theorey length: {theory_length_map}")
            # self.real.get_logger().info(f"Up width: {up_width}, Down width: {down_width}")

            def func(angle): 	#求解方程
                lengh_massure_1 = []	 #储存上激光的信息 
                for [dis, id] in up_width:  	# 扫描上激光的信息
                    lengh_massure_1.append(abs(dis * np.sin(angle + LASER_ANGLES[id])))  # 点到上边界的距离
                mean_lengh_massure_1 = np.mean(lengh_massure_1)
                lengh_massure_2 = [] #储存下激光的信息
                for [dis, id] in down_width:  # 扫描下激光的信息
                    lengh_massure_2.append(abs(dis * np.sin(angle + LASER_ANGLES[id])))  # 点到下边界的距离
                mean_lengh_massure_2 = np.mean(lengh_massure_2)
                return mean_lengh_massure_1 + mean_lengh_massure_2 - self.real.FIELD_SIZE[1]  # 返回方程的值

            laser_est_yaw = fsolve(func, x0=self.est_yaw)[0]  # x0 是浮点初始猜测
            # laser_est_yaw %= 2 * np.pi

            # self.real.get_logger().info(f"func(0): {func(0)}, func(laser_est_yaw): {func(laser_est_yaw)}")
            
            for i in range(len(LASER_ANGLES)): #用求解出的角度计算出坐标
                if flag[i] == 1: #数据无效
                    continue
                else:
                    single_yaw = (LASER_ANGLES[i] + laser_est_yaw)
                    if single_yaw == 0:#当激光打在右边界上时
                        laser_x.append(self.real.FIELD_SIZE[0] - data[i])
                    elif single_yaw == np.pi:#当激光打在左边界上时	
                        laser_x.append(data[i])
                    elif single_yaw == np.pi / 2:#当激光打在上边界上时
                        laser_y.append(self.real.FIELD_SIZE[1] - data[i])
                    elif single_yaw == np.pi * 3 / 2:#当激光打在下边界上时
                        laser_y.append(data[i])
                    else:
                        d_x = (self.real.FIELD_SIZE[0] - self.est_pos[0]) / np.cos(single_yaw) if np.cos(single_yaw) > 0 else -self.est_pos[0] / np.cos(single_yaw)
                        d_y = (self.real.FIELD_SIZE[1] - self.est_pos[1]) / np.sin(single_yaw) if np.sin(single_yaw) > 0 else -self.est_pos[1] / np.sin(single_yaw)
                        if d_x < d_y:
                            laser_x.append(self.real.FIELD_SIZE[0] - data[i] * np.cos(single_yaw) if np.cos(single_yaw) > 0 else -data[i] * np.cos(single_yaw))
                        else:
                            laser_y.append(self.real.FIELD_SIZE[1] - data[i] * np.sin(single_yaw) if np.sin(single_yaw) > 0 else -data[i] * np.sin(single_yaw))
            
            # self.real.get_logger().info(f"Data before Laser update: pos({self.est_pos[0]:.4f}, {self.est_pos[1]:.4f}), yaw({self.est_yaw:.4f})")

            LASER_data_flag = 1

            if len(laser_x) == 0:
                laser_x.append(self.est_pos[0])
                LASER_data_flag = 0
            if len(laser_y) == 0:
                laser_y.append(self.est_pos[1])
                LASER_data_flag = 0

            final_x = np.mean(laser_x)
            final_y = np.mean(laser_y)

            # if data == [-1] * len(LASER_ANGLES):
            #     LASER_data_flag = 1
            # else:
            #     LASER_data_flag = 1
            
            # self.real.get_logger().info(f"Laser: flag: {flag}, data: {data}, up_width: {up_width}, down_width: {down_width}, estimate: ({final_x:.4f}, {final_y:.4f}), angle: {laser_est_yaw:.4f}")
            self.real.get_logger().info(f"Laser: position (x: {final_x:.4f}, y: {final_y:.4f}), laser_est_yaw: {laser_est_yaw:.4f}")

            return [final_x, final_y], self.est_yaw

    class KalmanFilter:

        def __init__(self,estrobot,real):
            self.estrobot=estrobot
            self.state_dim = 5
            self.real=real
            self.last_time = time.time()
            self.initial_state_vector=[0.0, 0.0, 0.0, 0.0, 0.0],
            self.initial_state_uncertainty=1.0

        def predict(self):
            dt = time.time() - self.last_time
            self.last_time = time.time()
            
            # Initialize state vector [x, x_vel, y, y_vel, angle]
            self.state = np.array(self.initial_state_vector, dtype=np.float64).reshape((5, 1))
            self.state[0],self.state[2] = [float(self.real.START_POINT[0]),float(self.real.START_POINT[1])]
            # 修改状态转移矩阵（6x6）
            self.F = np.array([
                [1, dt, 0, 0,  0],
                [0, 1,  0, 0,  0],
                [0, 0,  1, dt, 0],
                [0, 0,  0, 1,  0],
                [0, 0,  0, 0,  1]
            ])

            # 过程噪声协方差Q（基于加速度噪声）
            q_x = (self.real.process_noise_std[0]**2) * np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
            q_y = (self.real.process_noise_std[1]**2) * np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
            q_angle = self.real.process_noise_std[2]**2 
            
            self.Q = np.zeros((5, 5))
            self.Q[0:2, 0:2] = q_x
            self.Q[2:4, 2:4] = q_y
            self.Q[4:5, 4:5] = q_angle
            # Measurement noise covariance (R) #测量噪声协方差
            # 测量噪声协方差R
            self.R = np.diag([self.real.measurement_noise_std[0]**2, 
                            self.real.measurement_noise_std[1]**2, 
                            self.real.measurement_noise_std[2]**2])

            # State covariance matrix (P) #状态协方差矩阵
            self.P = np.eye(5) * self.initial_state_uncertainty
            # Measurement matrix (H) (We are only measuring position and angle [x, y, angle]) 
            # #测量矩阵 (我们只测量位置和角度 [x, y, angle],使用激光雷达数据)
            self.H = np.array([[1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1]])
            self.B = np.array([
                [0.5*dt**2, 0],
                [dt,        0],
                [0, 0.5*dt**2],
                [0,        dt],
                [0,         0],
            ])

            acc_global = self.estrobot.acc_global  # 实现坐标转换
            ax, ay = acc_global[0], acc_global[1]
            # 更新位置
            self.state[0] += self.state[1] * dt + 0.5 * ax * dt**2
            self.state[2] += self.state[3] * dt + 0.5 * ay * dt**2
            # 应用加速度
            self.state[1] += ax * dt  # x方向速度
            self.state[3] += ay * dt  # y方向速度
            # 更新协方差
            self.P = self.F @ self.P @ self.F.T + self.Q

        def update(self):
            global LASER_data_flag
            # self.estrobot.update_imu()
            laser_position, laser_position_angle = self.estrobot.update_laser()

            if LASER_data_flag == 1:
                LASER_data_flag = 0
                # Measurement update using GPS position data (x, y, angle)
                laser_position_x, laser_position_y = laser_position[0], laser_position[1]
                z = np.array([laser_position_x, laser_position_y, laser_position_angle]).reshape((3, 1))
                # 测量更新使用激光位置数据 (x, y, angle)
                # self.real.get_logger().info(f"z: {z}\n")
                # Kalman Gain (K)
                S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R #预测误差协方差矩阵 
                K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S)) #卡尔曼增益

                # Update the state with the Laser measurement
                #卡尔曼滤波融合IMU和激光雷达数据
                self.state = self.state + np.dot(K, (z - np.dot(self.H, self.state)))

                # Update state covariance matrix
                #更新状态协方差矩阵
                I = np.eye(self.state_dim)
                self.P = np.dot(I - np.dot(K, self.H), self.P)

                self.final_state = [self.state[0], self.state[2], self.state[4]]
                self.estrobot.est_vel_x = self.state[1, 0]  # 提取标量
                self.estrobot.est_vel_y = self.state[3, 0]  # 提取标量
                self.estrobot.est_pos = np.array([self.state[0, 0], self.state[2, 0]])
                self.estrobot.est_yaw = self.state[4, 0] % (np.pi*2) # 提取标量
                # print('hhhhhhhhhhhhhhh')
                self.real.get_logger().info(f"self.final_state: {self.final_state}")
                self.real.get_logger().info(f"Kalman: pos({self.estrobot.est_pos[0]:.8f}, {self.estrobot.est_pos[1]:.4f}), vel({self.estrobot.est_vel_x:.4f}, {self.estrobot.est_vel_y:.4f}), yaw({self.estrobot.est_yaw:.4f})\n")

                if self.estrobot.est_pos[0] > self.real.FIELD_SIZE[0] or self.estrobot.est_pos[1] > self.real.FIELD_SIZE[1]:
                    self.real.get_logger().info(f"Out of bounds: pos({self.estrobot.est_pos[0]:.4f}, {self.estrobot.est_pos[1]:.4f})\n")

                return self.final_state

            else:#如果没有LASER数据，使用前IMU的数据进行遗传
                # Prediction step using IMU position data (x, y, angle)
                
                # Predict state assuming the new position is related to velocity#假设新位置与速度有关
                # We assume constant velocity model in between IMU position updates
                self.state[0] = self.estrobot.est_pos[0]  
                #将预测的x位置设置为IMU的位置
                self.state[2] = self.estrobot.est_pos[1]
                #将预测的y位置设置为IMU的位置
                self.state[4] = self.estrobot.est_yaw % np.pi / 2 # Set the predicted angle position to the IMU's position
                #将预测的角度位置设置为IMU的位置
                self.final_state = [self.state[0], self.state[2], self.state[4]]
                self.end_time = time.time()
                
                self.real.get_logger().info(f"Kalman (NO LASER): pos({self.estrobot.est_pos[0]:.4f}, {self.estrobot.est_pos[1]:.4f}), vel({self.estrobot.est_vel_x:.4f}, {self.estrobot.est_vel_y:.4f}), yaw({self.estrobot.est_yaw:.4f})")

                return self.final_state

    def __init__(self):
        super().__init__('laser_position_node')
        #就首激光雷达数据
        self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.scan_callback,
            10)
        
        self.real_laser=[-1] * len(LASER_ANGLES)
        #接受IMU发送的四元数信息
        self.create_subscription(
            QuaternionStamped,
            'imu/quaternion',
            self.quat_callback,
            10)
        #接受IMU发送的加速度信息
        self.create_subscription(
            Vector3Stamped,
            'imu/acceleration',
            self.accel_callback,
            10
        )
        # hasattr(self,"initial_angle")#判断是否有初始角度
        

        self.publisher_=self.create_publisher(PoseStamped,'/ally/robot1/laser_position_node',10)#发布激光位置数据，发布到/ally/robot1/laser_position，队列长度为10，发布的数据类型为LaserPosition，LaserPosition是自定义的数据类型

        self.yaw = 0.0  # 初始化 yaw_rate
        self.acc_x = 0.0  # 初始化 x 方向加速度
        self.acc_y = 0.0  # 初始化 y 方向加速度 # 初始化 yaw
        self.declare_parameters(
            namespace='',
            parameters=[
                ('FIELD_SIZE', [15.00,8.00]),
                ('process_noise_std', [0.1,0.1,0.05]),
                ('measurement_noise_std', [0.1,0.1,0.05]),
                ('START_POINT', [7.5,4.0]),
                ('LASER_ALLOWED_NOISE', 0.1),
                ('FREQUENCY', 1.00),
                ('DELTA_DISTANCE', 0.01),
                ('IMU_DISTANCE',0.1),
                ('GRAVITY', 9.7887)
            ]
        )
        # # 获取参数值（覆盖默认值）
        
        self.FIELD_SIZE = self.get_parameter('FIELD_SIZE').get_parameter_value().double_array_value
        self.process_noise_std = self.get_parameter('process_noise_std').get_parameter_value().double_array_value
        self.measurement_noise_std = self.get_parameter('measurement_noise_std').get_parameter_value().double_array_value
        self.START_POINT = self.get_parameter('START_POINT').get_parameter_value().double_array_value
        self.LASER_ALLOWED_NOISE = self.get_parameter('LASER_ALLOWED_NOISE').get_parameter_value().double_value
        self.FREQUENCY = self.get_parameter('FREQUENCY').get_parameter_value().double_value
        self.DELTA_DISTANCE = self.get_parameter('DELTA_DISTANCE').get_parameter_value().double_value
        self.IMU_DISTANCE = self.get_parameter('IMU_DISTANCE').get_parameter_value().double_value
        self.GRAVITY = self.get_parameter('GRAVITY').get_parameter_value().double_value

        # 打印参数值（调试用）
        self.get_logger().info(f"""
        Loaded Parameters:
        - FIELD_SIZE = {self.FIELD_SIZE}
        - process_noise_std = {self.process_noise_std}
        - measurement_noise_std = {self.measurement_noise_std}
        - START_POINT = {self.START_POINT}
        - LASER_ALLOWED_NOISE = {self.LASER_ALLOWED_NOISE}
        - FREQUENCY = {self.FREQUENCY}
        - DELTA_DISTANCE = {self.DELTA_DISTANCE}
        """)

        # 初始化 EstRobot 和 KalmanFilter
        self.est_robot = self.EstRobot(self)
        self.kalman_filter = self.KalmanFilter(self.est_robot, self)

        self.timer=self.create_timer(self.FREQUENCY,self.timer_callback)

        self.imu_print_counter = 0
    
    def scan_callback(self, msg):
        # 获取激光雷达数据并且进行可视化，使用matplotlib
        # self.get_logger().info('Laser scan received')
        # 计算距离最近的障碍物距离
        #开始计时
        
        self.real_laser = np.array(msg.data)
        # self.get_logger().info(f"Real laser data: {self.real_laser}\n")
    
    def quat_callback(self, msg):
        """四元数转欧拉角处理"""
        # 提取四元数
        quaternion = [
            msg.quaternion.x,
            msg.quaternion.y,
            msg.quaternion.z,
            msg.quaternion.w
        ]
        
        # 转换为欧拉角（弧度）
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        
        # 弧度
        self.euler_angles = [
            roll,
            pitch,
            yaw
        ]
        # 更新偏航角
        self.yaw = self.euler_angles[2] + 0.003

    def accel_callback(self, msg):
        """加速度数据处理"""
        self.acceleration = [
            msg.vector.x,
            msg.vector.y,
            msg.vector.z
        ]
        
        # self.get_logger().info(
        #     f"加速度[m/s²] - X: {self.acceleration[0]:.2f}, "
        #     f"Y: {self.acceleration[1]:.2f}, "
        #     f"Z: {self.acceleration[2]:.2f}"
        # )

        self.acc_x = self.acceleration[0] + 0.1313
        self.acc_y = self.acceleration[1] - 0.0490

        # 调用 update_imu 更新
        self.est_robot.update_imu()

    # def imu_callback(self, msg):
    #     # 从消息中获取四元数
    #     # time_1 = time.time()
    #     # self.get_logger().info(f"duration:{time_1-self.time2}")
    #     orientation_q = msg.orientation
    #     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #     # 将四元数转换为欧拉角
    #     # 获取角速度
    #     angular_velocity = msg.angular_velocity
    #     self.yaw_rate = angular_velocity.z  # 修正 yaw_rate 的获取方式
    #     # self.get_logger().info(f"=========Yaw rate from IMU: {self.yaw_rate}\n")
    #   
    #     # 获取加速度
    #     linear_acceleration = msg.linear_acceleration
    #     self.acc_x = -linear_acceleration.x * self.GRAVITY
    #     self.acc_y = -linear_acceleration.y * self.GRAVITY
    #     # (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
    #     # 打印 IMU 数据
    #     self.imu_print_counter += 1
    #     if self.imu_print_counter == 40:
    #         self.get_logger().info(f"Acceleration - x: {self.acc_x:.6f}, y: {self.acc_y:.6f}, z: {linear_acceleration.z:.4f}")
    #         self.get_logger().info(f"Yaw rate: {self.yaw_rate:.6f}")
    #         self.imu_print_counter = 0
    #     # self.time2 = time.time()
    #     self.est_robot.update_imu()  # 更新 IMU 数据
    
    def timer_callback(self):
        self.time_start = time.time()
        self.kalman_filter.predict()
        statement = self.kalman_filter.update()
        self.time_end = time.time()
        # 确保 statement 有效
        if statement is None:
            statement = [self.kalman_filter.state[0, 0], 
                         self.kalman_filter.state[2, 0], 
                         self.kalman_filter.state[4, 0]]
        # 发布消息
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        #机器人坐标及姿态
        msg.pose.position.x = -float(statement[0]) + self.FIELD_SIZE[0]
        msg.pose.position.y = -float(statement[1]) + self.FIELD_SIZE[1]
        msg.pose.position.z = 0.00

        #将欧拉角转换成四元数
        roll = 0.0
        pitch = 0.0
        yaw = float(statement[2])
        q = quaternion_from_euler(roll, pitch, yaw)

        #将四元数填充到消息中
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        # self.get_logger().info(f"{self.time_end - self.time_start}")
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published: {msg.x}, {msg.y}, {msg.angle}")

def main(args=None):
    # args = remove_ros_args(args)
    rclpy.init(args=args)
    real_node = Real()
    # est_robot = EstRobot(real_node)
    # kalman_filter = KalmanFilter(est_robot,real_node)
    # laser_position = Laserposition(kalman_filter,real_node)
    # # # 独立ROS线程
    # # ros_spin_thread = threading.Thread(target=rclpy.spin, args=(real_node,))
    # # ros_spin_thread.daemon = True
    # # ros_spin_thread.start()
    executor = MultiThreadedExecutor(num_threads=4)
    # executor.add_node(laser_position)
    executor.add_node(real_node)

    try:
        
        # while True:
        # 	# 固定频率10Hz
        # 	time.sleep(0.05)
            
        # 	# 强制更新（移除时间戳检查）
        # 	est_robot.update_imu()    # 确保IMU更新
        # 	est_robot.update_laser()  # 确保激光更新
            
        # 	# 卡尔曼滤波
        # 	kalman_filter.predict()
        # 	kalman_filter.update()
        executor.spin()

    except KeyboardInterrupt:
        # 退出

        executor.shutdown()
        rclpy.shutdown()
            
if __name__ == '__main__':
    main()  

    ##Question 1 why the velocity will increase when the laser 1 is zhedanged"
    ##Question 2 why the angle is 0 ,i think it at least will show a wrong number"
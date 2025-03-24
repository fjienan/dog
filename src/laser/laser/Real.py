import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from laser_position.msg._laser_position import LaserPosition 
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import threading
from scipy.optimize import fsolve
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
# from laser_interfaces.msg import LaserPosition
# 设置字体为 SimHei 以支持中文显示
# rcParams['font.sans-serif'] = ['SimHei']
# rcParams['axes.unicode_minus'] = False

# 模拟参数
LASER_data_flag = 1		# 是否有LASER	数据
# self.real.DT = 0.05                    # IMU 时间步长
# FIELD_SIZE = [15, 8]           # 场地大小
LASER_ANGLES = [0, np.deg2rad(72), np.deg2rad(144), np.deg2rad(216), np.deg2rad(288)]
# self.real.precess_noise_std = [0.1, 0.1, 0.05],
# self.real.measurement_noise_std=[0.5,0.5,0.1], 
# self.real.START_POINT = [7.5, 4]       # 初始位置
# LASER_ALLOWED_NOISE = 2      # 激光测量允许的噪声


class Real(Node):
    def __init__(self):
        super().__init__('laser_position_1')
        #
        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/ally/robot1/scan',
            self.scan_callback,
            10)
        self.subscription_laser  # 防止未使用的变量警告
        self.real_laser=[-1] * len(LASER_ANGLES)
        self.subscription_imu = self.create_subscription(
            Imu,
            '/ally/robot1/imu',
            self.listener_callback,
            10)
        self.subscriptions  # 防止未使用的变量警告
        self.yaw_rate = 0.0  # 初始化 yaw_rate
        self.acc_x = 0.0  # 初始化 x 方向加速度
        self.acc_y = 0.0  # 初始化 y 方向加速度 # 初始化 yaw
        self.declare_parameters(
            namespace='',
            parameters=[
                ('DT',0.1),
                ('FIELD_SIZE',[15,8]),
                ('process_noise_std',[0.1,0.1,0.05]),
                ('measurement_noise_std',[0.1,0.1,0.05]),
                ('START_POINT',[7.5,4.0]),
                ('LASER_ALLOWED_NOISE',0.1),
                ('FREQUENCY',1.00)
            ]
        )
        # # 获取参数值（覆盖默认值）
        
        self.DT = self.get_parameter('DT').get_parameter_value().double_value
        self.FIELD_SIZE = self.get_parameter('FIELD_SIZE').get_parameter_value().integer_array_value
        self.process_noise_std = self.get_parameter('process_noise_std').get_parameter_value().double_array_value
        self.measurement_noise_std = self.get_parameter('measurement_noise_std').get_parameter_value().double_array_value
        self.START_POINT = self.get_parameter('START_POINT').get_parameter_value().double_array_value
        self.LASER_ALLOWED_NOISE = self.get_parameter('LASER_ALLOWED_NOISE').get_parameter_value().double_value
        self.FREQUENCY = self.get_parameter('FREQUENCY').get_parameter_value().double_value

        # 打印参数值（调试用）
        self.get_logger().info(f"""
        Loaded Parameters:
        - DT = {self.DT}
        - FIELD_SIZE = {self.FIELD_SIZE}
        - process_noise_std = {self.process_noise_std}
        - measurement_noise_std = {self.measurement_noise_std}
        - START_POINT = {self.START_POINT}
        - LASER_ALLOWED_NOISE = {self.LASER_ALLOWED_NOISE}
        - FREQUENCY = {self.FREQUENCY}
        """)
       
    def scan_callback(self, msg):
        # 获取激光雷达数据并且进行可视化，使用matplotlib
        # self.get_logger().info('Laser scan received')
        # 计算距离最近的障碍物距离
        #开始计时
        
        ranges = np.array(msg.ranges)
        self.real_laser = [ranges[0], ranges[72], ranges[144], ranges[216], ranges[288]]
        # self.get_logger().info(f"Real laser data: {self.real_laser}\n")
    def listener_callback(self, msg):
        # 从消息中获取四元数
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # 将四元数转换为欧拉角
        # 获取角速度
        angular_velocity = msg.angular_velocity
        self.yaw_rate = angular_velocity.z  # 修正 yaw_rate 的获取方式
        # self.get_logger().info(f"=========Yaw rate from IMU: {self.yaw_rate}\n")
        # # 获取角加速度
        # yaw_acc = msg.angular_acceleration
        # self.yaw_acc = yaw_acc
        # self.get_logger().info(f"Yaw acceleration from IMU: {self.yaw_acc}")
        # 获取加速度
        linear_acceleration = msg.linear_acceleration
        self.acc_x = linear_acceleration.x
        self.acc_y = linear_acceleration.y
        # self.get_logger().info(f"Acceleration - x: {self.acc_x}, y: {self.acc_y}\n")
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
        # 打印 yaw 角
        # self.get_logger().info(f"Yaw angle from IMU: {self.yaw}\n")

class EstRobot:
    def __init__(self, real):
        self.lock = threading.Lock()
        self.real = real
        self.est_yaw = 0.0 # 估计朝向
        self.est_pos = np.array(self.real.START_POINT, dtype=np.float64)  # 估计位置
        self.est_vel_x = 0.0
        self.est_vel_y = 0.0
        # self.acc_body = np.array([self.real.acc_x, self.real.acc_y]).reshape((2, 1))
        self.acc_body = np.zeros((2, 1))  # 初始化为零
        self.acc_global = np.zeros((2, 1))  # 新增必要属性
        self.start_time = 0
    def update_imu(self):
        # print(f"Data before IMU update: pos({self.est_pos[0]:.4f}, {self.est_pos[1]:.4f}), vel({self.est_vel_x:.4f}, {self.est_vel_y:.4f}), yaw({self.est_yaw:.4f})")
        self.start_time = time.time()
        current_acc_x = float(self.real.acc_x)
        current_acc_y = float(self.real.acc_y)
        current_yaw_rate = float(self.real.yaw_rate)

        print(f"IMU current_acc_x: {current_acc_x}, current_acc_y: {current_acc_y}, current_yaw_rate: {current_yaw_rate}\n")

        # 更新角速度积分
        self.est_yaw += current_yaw_rate * self.real.DT
        # self.est_yaw %= 2 * np.pi
        
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
        self.acc_body = np.array([[current_acc_x], [current_acc_y]], dtype=np.float64)
        
        # 检查矩阵维度
        assert self.acc_body.shape == (2, 1), f"acc_body维度错误: {self.acc_body.shape}"

        # 坐标变换
        # R = np.array([
        # 	[np.cos(self.est_yaw), -np.sin(self.est_yaw)],
        # 	[np.sin(self.est_yaw), np.cos(self.est_yaw)]
        # ], dtype=np.float64)

        assert R.shape == (2, 2), f"旋转矩阵维度错误: {R.shape}"

        self.acc_global = R @ self.acc_body

        # 运动学更新
        self.est_vel_x += self.acc_global[0, 0] * self.real.DT
        self.est_vel_y += self.acc_global[1, 0] * self.real.DT
        print (f"IMU x_vel: {self.est_vel_x}, y_vel: {self.est_vel_y}")
        self.est_pos[0] += self.est_vel_x * self.real.DT
        self.est_pos[1] += self.est_vel_y * self.real.DT

    def update_laser(self):
        global LASER_data_flag
        laser_data = self.real.real_laser.copy()

        # 过滤无效数据 (NaN/inf)
        laser_data = np.nan_to_num(laser_data, nan=-1, posinf=self.real.FIELD_SIZE[0], neginf=0.0)
        
        laser_x = []
        laser_y = []
        data = []
        up_width = []
        down_width = []
        flag = [0] * len(LASER_ANGLES)
        theory_length_map=[0] *len(LASER_ANGLES)
        # 计算激光的理论长度
        for i in range(len(LASER_ANGLES)): #计算激光的理论长度
            data.append(laser_data[i]) #获取每个激光的数据
            if data[i] == -1:
                flag[i] = 1
                continue
            laser_yaw = (LASER_ANGLES[i] + self.est_yaw) # % (2 * np.pi)
            if laser_yaw == 0: 		#当激光打在右边界上时
                thorey_length = self.real.FIELD_SIZE[0] - self.est_pos[0]
                theory_length_map[i]=thorey_length
            elif laser_yaw == np.pi: 		#当激光打在左边界上时
                thorey_length =  self.est_pos[0]
                theory_length_map[i]=thorey_length
            elif laser_yaw == np.pi / 2: 		#当激光打在上边界上时
                thorey_length =  self.real.FIELD_SIZE[1] - self.est_pos[1]
                theory_length_map[i]=thorey_length
            elif laser_yaw == -np.pi / 2:		 #当激光打在下边界上时
                thorey_length =  self.est_pos[1]
                theory_length_map[i]=thorey_length
            else: 		#当激光打在其他地方时
                d_x = (self.real.FIELD_SIZE[0] - self.est_pos[0]) / np.cos(laser_yaw) if np.cos(laser_yaw) > 0 else -self.est_pos[0] / np.cos(laser_yaw)
                d_y = (self.real.FIELD_SIZE[1] - self.est_pos[1]) / np.sin(laser_yaw) if np.sin(laser_yaw) > 0 else -self.est_pos[1] / np.sin(laser_yaw)
                thorey_length = d_x if d_x < d_y else d_y
                theory_length_map[i]=thorey_length
                if abs(data[i] - thorey_length) <= self.real.LASER_ALLOWED_NOISE and d_x > d_y:
                    if np.sin(laser_yaw) > 0:
                        up_width.append([data[i], i])
                    else:
                        down_width.append([data[i], i])
                elif abs(data[i] - thorey_length) > self.real.LASER_ALLOWED_NOISE: #当激光数据与理论数据差距大于允许的噪声时,认为激光数据无效
                    flag[i] = 1 	#-1代表激光数据无效
        # print(f"theoretical length: {theory_length_map}")

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
        
        for i in range(len(LASER_ANGLES)): #用求解出的角度计算出坐标
            if flag[i] == 1: #数据无效
                continue
            else:
                single_yaw = (LASER_ANGLES[i] + laser_est_yaw) % (2 * np.pi)
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

        # print(f"Data before Laser update: pos({self.est_pos[0]:.4f}, {self.est_pos[1]:.4f}), yaw({self.est_yaw:.4f})")
        if len(laser_x) == 0:
            laser_x.append(self.est_pos[0])
            print(" ^v^ ^v^ ^v^ ^v^ ^v^ in this place,the laser cann`t detect the obstacle precisely")
            LASER_data_flag = 0
        if len(laser_y) == 0:
            laser_y.append(self.est_pos[1])
            print(" ^v^ ^v^ ^v^ ^v^ ^v^ in this place,the laser cann`t detect the obstacle precisely")
            LASER_data_flag = 0

        final_x = np.mean(laser_x)
        final_y = np.mean(laser_y)
        print(f"Laser valid flag: {flag}")
        # print(f"Laser data: {laser_data}")
        # print(f"Laser estimate: ({final_x:.4f}, {final_y:.4f}), angle: {laser_est_yaw:.4f}\n")
        # print(f"  Laser_x: {laser_x}, Laser_y: {laser_y}\n")
        if data == [-1] * len(LASER_ANGLES):
            LASER_data_flag = 1
        else :
            LASER_data_flag = 1
        return [final_x, final_y], self.est_yaw

############################################
'卡尔曼滤波器'
############################################
class KalmanFilter:

    def __init__(self,estrobot,real, # [ax, ay, angular_acc],   
                 initial_state_vector=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                 initial_state_uncertainty=1.0
                 ):
        self.estrobot=estrobot
        self.state_dim = 6
        self.real=real 
        self.end_time = 0.0
        
        # Initialize state vector [x, x_vel, y, y_vel, angle, angular_vel]
        self.state = np.array(initial_state_vector, dtype=np.float64).reshape((6, 1))
        self.state[0],self.state[2] = [float(self.real.START_POINT[0]),float(self.real.START_POINT[1])]
        # 修改状态转移矩阵（6x6）
        self.F = np.array([
            [1, self.real.DT, 0, 0,  0, 0],
            [0, 1,  0, 0,  0, 0],
            [0, 0,  1, self.real.DT, 0, 0],
            [0, 0,  0, 1,  0, 0],
            [0, 0,  0, 0,  1, self.real.DT],
            [0, 0,  0, 0,  0, 1]
        ])

        # 过程噪声协方差Q（基于加速度噪声）
        q_x = (self.real.process_noise_std[0]**2) * np.array([[self.real.DT**4/4, self.real.DT**3/2], [self.real.DT**3/2, self.real.DT**2]])
        q_y = (self.real.process_noise_std[1]**2) * np.array([[self.real.DT**4/4, self.real.DT**3/2], [self.real.DT**3/2, self.real.DT**2]])
        q_angle = (self.real.process_noise_std[2]**2) * np.array([[self.real.DT**4/4, self.real.DT**3/2], [self.real.DT**3/2, self.real.DT**2]])
        
        self.Q = np.zeros((6, 6))
        self.Q[0:2, 0:2] = q_x
        self.Q[2:4, 2:4] = q_y
        self.Q[4:6, 4:6] = q_angle
        # Measurement noise covariance (R) #测量噪声协方差
        # 测量噪声协方差R
        self.R = np.diag([self.real.measurement_noise_std[0]**2, 
                         self.real.measurement_noise_std[1]**2, 
                         self.real.measurement_noise_std[2]**2])

        # State covariance matrix (P) #状态协方差矩阵
        self.P = np.eye(6) * initial_state_uncertainty
        # Measurement matrix (H) (We are only measuring position and angle [x, y, angle]) 
        # #测量矩阵 (我们只测量位置和角度 [x, y, angle],使用激光雷达数据)
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0]])
        self.B = np.array([
            [0.5*self.real.DT**2, 0],
            [self.real.DT,        0],
            [0,         0.5*self.real.DT**2],
            [0,         self.real.DT],
            [0,         0],
            [0,         0]
        ])
    def predict(self):
        ax = self.estrobot.acc_global[0, 0]
        ay = self.estrobot.acc_global[1, 0]
        acc_global = self.estrobot.acc_global  # 实现坐标转换
        ax, ay = acc_global[0], acc_global[1]
        # 更新位置
        self.state[0] += self.state[1] * self.real.DT + 0.5 * ax * self.real.DT**2
        self.state[2] += self.state[3] * self.real.DT + 0.5 * ay * self.real.DT**2
        self.state[4] += self.state[5] * self.real.DT # Update angle position
        # 应用加速度
        self.state[1] += ax * self.real.DT  # x方向速度
        self.state[3] += ay * self.real.DT  # y方向速度
        # 更新协方差
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self):
        if LASER_data_flag == 1:
            
        # Measurement update using GPS position data (x, y, angle)
         
            laser_position, laser_position_angle = self.estrobot.update_laser()
            laser_position_x, laser_position_y = laser_position[0], laser_position[1]
            z = np.array([laser_position_x, laser_position_y, laser_position_angle]).reshape((3, 1))
            # 测量更新使用激光位置数据 (x, y, angle)

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
            self.estrobot.est_yaw = self.state[4, 0] % np.pi/2 # 提取标量
            print('hhhhhhhhhhhhhhh')
            print(f"Kalman: pos({self.estrobot.est_pos[0]:.1f}, {self.estrobot.est_pos[1]:.1f}), vel({self.estrobot.est_vel_x:.1f}, {self.estrobot.est_vel_y:.1f}), yaw({self.estrobot.est_yaw:.1f})\n")
            self.end_time = time.time()
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
            print("hahahahahahahhahahahah")
            return self.final_state
        
    
class Laserposition(Node):

    def __init__(self,kalman_filter,real_node):
        super().__init__('laser_position_node')
        self.publisher_=self.create_publisher(LaserPosition,'/ally/robot1/laser_position_node',10)#发布激光位置数据，发布到/ally/robot1/laser_position，队列长度为10，发布的数据类型为LaserPosition，LaserPosition是自定义的数据类型
        self.kalman_filter = kalman_filter
        self.real = real_node
        time_period= self.real.FREQUENCY #`发布激光位置数据的时间间隔  20Hz`
        self.timer=self.create_timer(time_period,self.timer_callback)

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
        msg = LaserPosition()
        msg.x = float(statement[0])
        msg.y = float(statement[1])
        msg.angle = float(statement[2])
        print(self.time_end - self.time_start)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.x}, {msg.y}, {msg.angle}")

def main(args=None):
    # args = remove_ros_args(args)
    rclpy.init(args=args)
    real_node = Real()
    est_robot = EstRobot(real_node)
    kalman_filter = KalmanFilter(est_robot,real_node)
    laser_position = Laserposition(kalman_filter,real_node)
    # # 独立ROS线程
    # ros_spin_thread = threading.Thread(target=rclpy.spin, args=(real_node,))
    # ros_spin_thread.daemon = True
    # ros_spin_thread.start()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(laser_position)
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
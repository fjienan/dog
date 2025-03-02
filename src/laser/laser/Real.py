import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib import rcParams, patches
import time
from scipy.optimize import fsolve
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
from laser_interfaces.msg import LaserPosition
# 设置字体为 SimHei 以支持中文显示
# rcParams['font.sans-serif'] = ['SimHei']
# rcParams['axes.unicode_minus'] = False

# 模拟参数
imu_data_flag = 1 			# 是否有IMU数据
R = 1.0                     # Meaurement noise value
Q = 1
DT = 0.02                      # IMU 时间步长
FIELD_SIZE = [15, 8]           # 场地大小
MAX_SPEED = 0.5                # 最大速度 m/s
LASER_ANGLES = [0, np.deg2rad(72), np.deg2rad(144), np.deg2rad(216), np.deg2rad(288)]
START_POINT = [7.5, 4.0]       # 初始位置
LASER_ALLOWED_NOISE = 0.3      # 激光测量允许的噪声

class MinimalPublisher(Node):
	def __init__(self,kalman_filter):
		self.kalman_filter=kalman_filter
		super().__init__('minimal_publisher')
		# 创建一个发布者，发布到'topic' topic，消息类型是String
		self.publisher_ = self.create_publisher(LaserPosition, 'laser_position', 10)
		# 设置定时器，每秒发布一次消息
		self.timer = self.create_timer(1.0, self.timer_callback)
		self.i = 0

	def timer_callback(self):
		msg = LaserPosition()
		final_state = self.kalman_filter.update()
		msg.x=float(final_state[0])
		msg.y=float(final_state[1])
		msg.angle=float(final_state[2])
		self.publisher_.publish(msg)
		self.get_logger().info(f'Publishing: {msg}')
		self.i += 1

def main(args=None):
	rclpy.init(args=args)
	
	real = Real()
	# laser_obstacle = Laser_obstacle(real,realrobot)
	estrobot = EstRobot(real)
	kalman_filter = KalmanFilter(estrobot, real)
	minimal_publisher = MinimalPublisher(kalman_filter)
	rclpy.spin(minimal_publisher)
	minimal_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

class Real(Node):
	def __init__(self):
		super().__init__('laser_position')
		
		self.subscription_laser = self.create_subscription(
			LaserScan,
			'/ally/robot1/scan',
			self.scan_callback,
			10)
		self.subscription_laser  # 防止未使用的变量警告
		self.real_laser=[0] * len(LASER_ANGLES)
		self.subscription_imu = self.create_subscription(
			Imu,
			'/ally/robot1/imu',
			self.listener_callback,
			10)
		self.subscriptions  # 防止未使用的变量警告
		
	def scan_callback(self, msg):
		# 获取激光雷达数据并且进行可视化，使用matplotlib
		self.get_logger().info('Laser scan received')
		# 计算距离最近的障碍物距离
		ranges = np.array(msg.ranges)
		self.real_laser = [ranges[0], ranges[72], ranges[144], ranges[216], ranges[288]]

	def listener_callback(self, msg):
		# 从消息中获取四元数
		orientation_q = msg.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		# 将四元数转换为欧拉角
		# 获取角速度
		angular_velocity = msg.angular_velocity
		self.yaw_rate = angular_velocity
		self.get_logger().info(f"Yaw rate from IMU: {self.yaw_rate}")
		# # 获取角加速度
		# yaw_acc = msg.angular_acceleration
		# self.yaw_acc = yaw_acc
		# self.get_logger().info(f"Yaw acceleration from IMU: {self.yaw_acc}")
		# 获取加速度
		linear_acceleration = msg.linear_acceleration
		self.acc_x = linear_acceleration.x
		self.acc_y = linear_acceleration.y
		self.get_logger().info(f"Acceleration - x: {self.acc_x}, y: {self.acc_y}")
		(roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
		# 打印 yaw 角
		self.get_logger().info(f"Yaw angle from IMU: {self.yaw}")

# class RealRobot: # 获取理论激光长度
# 	def __init__(self):
# 		# 真实状态
# 		self.yaw = 0.0                      # 初始朝向（弧度）
# 		self.pos = np.array(START_POINT)    # 初始位置 (x,y)
# 		self.vel = 0.0                      # 初始速度
# 		# 控制参数
# 		self.acceleration = 0.3             # 加速度 m/s²
# 		self.turning = 0.0                  # 转向速度
# 		self.acc = 0.0                      # 当前加速度
# 	def get_laser(self, id):
# 		laser_yaw = (LASER_ANGLES[id] + self.yaw) % (2 * np.pi)
# 		if laser_yaw == 0:
# 			return FIELD_SIZE[0] - self.pos[0] + np.random.normal(0, self.LASER_NOISE)
# 		elif laser_yaw == np.pi:
# 			return self.pos[0] + np.random.normal(0, self.LASER_NOISE)
# 		elif laser_yaw == np.pi / 2:
# 			return FIELD_SIZE[1] - self.pos[1] + np.random.normal(0, self.LASER_NOISE)
# 		elif laser_yaw == -np.pi / 2:
# 			return self.pos[1] + np.random.normal(0, self.LASER_NOISE)
# 		else:
# 			d_x = (FIELD_SIZE[0] - self.pos[0]) / np.cos(laser_yaw) if np.cos(laser_yaw) > 0 else -self.pos[0] / np.cos(laser_yaw)
# 			d_y = (FIELD_SIZE[1] - self.pos[1]) / np.sin(laser_yaw) if np.sin(laser_yaw) > 0 else -self.pos[1] / np.sin(laser_yaw)
# 			return d_x if d_x < d_y else d_y

# 	def get_time(self):
# 		return time.time()

############################################
class EstRobot:
	def __init__(self, real):
		self.real = real
		self.est_yaw = self.kalman_filter.fianl_state[2]  # 估计朝向
		self.est_pos = np.array(START_POINT)  # 估计位置
		self.est_vel = 0.0

	def update_imu(self, acc_body, yaw_acc, dt):
		# 更新角速度（角加速度积分）
		self.yaw_rate += yaw_acc * dt

		# 更新朝向（角速度积分）
		self.est_yaw += self.yaw_rate * dt
		self.est_yaw %= 2 * np.pi

		# 将本体系加速度转换到全局坐标系
		R = np.array([[np.cos(self.est_yaw), -np.sin(self.est_yaw)], 
					[np.sin(self.est_yaw), np.cos(self.est_yaw)]])
		acc_global = R @ acc_body
		ax_global, ay_global = acc_global[0], acc_global[1]

		# 更新全局速度
		self.est_vel += np.sqrt(ax_global**2 + ay_global**2) * dt

		# 更新全局位置
		self.est_pos[0] += self.est_vel * np.cos(self.est_yaw) * dt
		self.est_pos[1] += self.est_vel * np.sin(self.est_yaw) * dt

	def update_laser(self):
		laser_x = []
		laser_y = []
		data = []
		up_width = []
		down_width = []
		flag = [0] * len(LASER_ANGLES)

		for i in range(len(LASER_ANGLES)):
			data.append(self.real.real_laser[i])
			laser_yaw = (LASER_ANGLES[i] + self.est_yaw) % (2 * np.pi)
			if laser_yaw == 0:
				thorey_length = FIELD_SIZE[0] - self.est_pos[0]
			elif laser_yaw == np.pi:
				thorey_length =  self.est_pos[0]
			elif laser_yaw == np.pi / 2:
				thorey_length =  FIELD_SIZE[1] - self.est_pos[1]
			elif laser_yaw == -np.pi / 2:
				thorey_length =  self.est_pos[1]
			else:
				d_x = (FIELD_SIZE[0] - self.est_pos[0]) / np.cos(laser_yaw) if np.cos(laser_yaw) > 0 else -self.est_pos[0] / np.cos(laser_yaw)
				d_y = (FIELD_SIZE[1] - self.est_pos[1]) / np.sin(laser_yaw) if np.sin(laser_yaw) > 0 else -self.est_pos[1] / np.sin(laser_yaw)
				thorey_length = d_x if d_x < d_y else d_y
				if abs(data[i] - thorey_length) <= LASER_ALLOWED_NOISE and d_x > d_y:
					if np.sin(laser_yaw) > 0:
						up_width.append([data[i], i])
					else:
						down_width.append([data[i], i])
			if abs(data[i] - thorey_length) > LASER_ALLOWED_NOISE:
				flag[i] = 1

		def func(angle):
			lengh_massure_1 = []
			for [dis, id] in up_width:  # 扫描上激光的信息
				lengh_massure_1.append(abs(dis * np.sin(angle + LASER_ANGLES[id])))  # 点到上边界的距离
			mean_lengh_massure_1 = np.mean(lengh_massure_1)
			lengh_massure_2 = []
			for [dis, id] in down_width:  # 扫描下激光的信息
				lengh_massure_2.append(abs(dis * np.sin(angle + LASER_ANGLES[id])))  # 点到下边界的距离
			mean_lengh_massure_2 = np.mean(lengh_massure_2)
			return mean_lengh_massure_1 + mean_lengh_massure_2 - FIELD_SIZE[1]

		laser_est_yaw = fsolve(func, x0=self.est_yaw)[0]  # x0 是浮点初始猜测
		laser_est_yaw %= 2 * np.pi

		for i in range(len(LASER_ANGLES)):
			if flag[i] == 1:
				continue
			else:
				single_yaw = (LASER_ANGLES[i] + laser_est_yaw) % (2 * np.pi)
				if single_yaw == 0:
					laser_x.append(FIELD_SIZE[0] - data[i])
				elif single_yaw == np.pi:
					laser_x.append(data[i])
				elif single_yaw == np.pi / 2:
					laser_y.append(FIELD_SIZE[1] - data[i])
				elif single_yaw == np.pi * 3 / 2:
					laser_y.append(data[i])
				else:
					d_x = (FIELD_SIZE[0] - self.est_pos[0]) / np.cos(single_yaw) if np.cos(single_yaw) > 0 else -self.est_pos[0] / np.cos(single_yaw)
					d_y = (FIELD_SIZE[1] - self.est_pos[1]) / np.sin(single_yaw) if np.sin(single_yaw) > 0 else -self.est_pos[1] / np.sin(single_yaw)
					if d_x < d_y:
						laser_x.append(FIELD_SIZE[0] - data[i] * np.cos(single_yaw) if np.cos(single_yaw) > 0 else -data[i] * np.cos(single_yaw))
					else:
						laser_y.append(FIELD_SIZE[1] - data[i] * np.sin(single_yaw) if np.sin(single_yaw) > 0 else -data[i] * np.sin(single_yaw))

		if len(laser_x) == 0:
			laser_x.append(self.est_pos[0])
		if len(laser_y) == 0:
			laser_y.append(self.est_pos[1])
			Q=6.0
			R=0.5
			print("in this place,the laser cann`t detect the obstacle precisely")

		return [np.mean(laser_x), np.mean(laser_y)], laser_est_yaw


############################################
'处理激光遮挡及动态调参'
############################################
# class Laser_obstacle:

# 	def __init__(self,real,realrobot):
# 		self.realrobot=realrobot
# 		self.real=real
# 		self.theory_length = []
# 		self.error=[]
# 		self.error_id=[]

# 	def laser_theory_length(self):
# 		self.real_length = self.real.real_laser
# 		for i in range(len(LASER_ANGLES)):
# 			self.theory_length.append(self.realrobot.get_laser(i))##到时候Realrobot类可以作为理论长度计算而保留
# 		for i in range(len(LASER_ANGLES)):
# 			error = abs(self.theory_length[i]-self.real_length[i])
# 			if error > 0.1:
# 				print ("Warning:Obstacle ",i," detected")
# 				self.error_id.append(i)
# 		return  self.error_id
		
############################################
'卡尔曼滤波器'
############################################
class KalmanFilter:

	def __init__(self,estrobot,real,dt=DT, process_noise_cov=np.eye(5), measurement_noise_cov=np.eye(3), state_cov=np.eye(5), initial_state_vector=[0, 0, 0, 0, 0],state_dim=5):
		self.estrobot=estrobot
		self.real=real
		self.dt = dt  # Time step
		
		# Initialize state vector [x, x_vel, y, y_vel, angle, angular_vel]
		self.state = np.array(initial_state_vector).reshape((5, 1))
	   
		# State transition matrix (F) #状态转移矩阵
		self.F = np.array([[1, dt, 0, 0,  0],
						   [0, 1,  0, 0,  0],
						   [0, 0,  1, dt, 0],
						   [0, 0,  0, 1,  0],
						   [0, 0,  0, 0,  1]])
		
		# Process noise covariance (Q) #过程噪声协方差
		self.Q = process_noise_cov
		# Measurement noise covariance (R) #测量噪声协方差
		self.R = measurement_noise_cov
		# State covariance matrix (P) #状态协方差矩阵
		self.P = state_cov
		# Measurement matrix (H) (We are only measuring position and angle [x, y, angle]) 
		# #测量矩阵 (我们只测量位置和角度 [x, y, angle],使用激光雷达数据)
		self.H = np.array([[1, 0, 0, 0, 0],
						   [0, 0, 1, 0, 0],
						   [0, 0, 0, 0, 1]])

	def predict(self):
	# 获取全局坐标系下的加速度
		if(globals.imu_data_flag==1):
			acc_global = self.get_global_acceleration()  # 实现坐标转换
			ax, ay = acc_global
			# 更新状态转移矩阵
			self.F[0, 1] = DT
			self.F[2, 3] = DT
			# 应用加速度
			self.state[1] += ax * DT  # x方向速度
			self.state[3] += ay * DT  # y方向速度
			# 更新位置
			self.state[0] += self.state[1] * DT + 0.5 * ax * DT**2
			self.state[2] += self.state[3] * DT + 0.5 * ay * DT**2
			self.state[4] += self.state[5] * self.dt # Update angle position
			# 更新协方差
			self.P = self.F @ self.P @ self.F.T + self.Q

		else:#如果没有IMU数据，使用激光数据进行预测
			# Prediction step using IMU position data (x, y, angle)
			laser_pos, laser_angle = self.estrobot.update_laser()
			imu_x, imu_y, imu_angle = laser_pos[0], laser_pos[1], laser_angle
			
			# Predict state assuming the new position is related to velocity#假设新位置与速度有关
			# We assume constant velocity model in between IMU position updates
			self.state[0] = imu_x  # Set the predicted x position to the IMU's position、
			#将预测的x位置设置为IMU的位置
			self.state[2] = imu_y  # Set the predicted y position to the IMU's position
			#将预测的y位置设置为IMU的位置
			self.state[4] = imu_angle  # Set the predicted angle position to the IMU's position
			#将预测的角度位置设置为IMU的位置
		return self.state

	def update(self):
		# Measurement update using GPS position data (x, y, angle)
		 
		laser_position, laser_position_angle = self.estrobot.update_laser()
		laser_position_x, laser_position_y = laser_position[0], laser_position[1]
		z = np.array([[laser_position_x], [laser_position_y], [laser_position_angle]])
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

		self.final_state = [self.state[0]*2, self.state[2]*2, self.state[4]*2]
	
		# 更新 estrobot 的状态
		self.estrobot.est_vel = np.sqrt(self.state[1]**2 + self.state[3]**2)
		self.estrobot.est_pos = np.array([self.final_state[0], self.final_state[1]])
		self.estrobot.est_yaw = self.final_state[2]

		# 返回最终状态
		return self.final_state

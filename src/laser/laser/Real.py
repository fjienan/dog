import rclpy
from rclpy.node import Node
# from laser_position.msg._laser_position import LaserPosition
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

import threading
from scipy.optimize import fsolve
# from sensor_msgs.msg import Imu
# from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray

LASER_data_flag = 1		# 是否有LASER	数据
# FIELD_SIZE = [15, 8]           # 场地大小
LASER_ANGLES = [np.deg2rad(60), np.deg2rad(132), np.deg2rad(204), np.deg2rad(276), np.deg2rad(348)]
# self.real.precess_noise_std = [0.1, 0.1, 0.05],
# self.real.measurement_noise_std=[0.5,0.5,0.1], 
# self.real.START_POINT = [7.5, 4]       # 初始位置
# LASER_ALLOWED_NOISE = 2      # 激光测量允许的噪声


class Real(Node):

    class EstRobot:
        def __init__(self, real):
            self.lock = threading.Lock()
            self.real = real
            self.est_yaw = 0.0 # 估计朝向
            self.est_pos = np.array(self.real.START_POINT, dtype=np.float64)  # 估计位置
            # self.acc_body = np.array([self.real.acc_x, self.real.acc_y]).reshape((2, 1))
            self.start_time = 0
            self.final_state = [0,0,0]


        def update_laser(self):
            #Import the data from the 3D laser
            self.est_yaw = self.real.yaw
            self.est_pos = self.real.odo_position
            self.real.get_logger().info(f"Laser: position (x: {self.est_pos[0]:.4f}, y: {self.est_pos[1]:.4f}), yaw: {self.est_yaw:.4f}")

            global LASER_data_flag
            # Get the laser data
            laser_data = self.real.real_laser.copy()

            # 过滤无效数据 (NaN/inf)
            laser_data = np.nan_to_num(laser_data, nan=-1, posinf=self.real.FIELD_SIZE[0], neginf=0.0)
            laser_data = [7.5 - 0.1247,4.206 - 0.1247,6.805 - 0.1247,6.805 - 0.1247,4.206 - 0.1247]
            # laser_data = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            #Check whether the laser data is valid
            if len(laser_data) != len(LASER_ANGLES):
                self.real.get_logger().info(f"!!!!!!!!!!!Laser data length mismatch: {len(laser_data)} != {len(LASER_ANGLES)}")
                LASER_data_flag = 0
                return None, None
            else:
                self.real.get_logger().info(f"!! Having !! Laser data: {laser_data}")
                LASER_data_flag = 1

            laser_x = []
            laser_y = []
            data = []
            up_width = []
            down_width = []
            flag = [0] * len(LASER_ANGLES)
            theory_length_map= [0] * len(LASER_ANGLES)
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
                # laser_x.append(self.est_pos[0])
                self.real.get_logger().info(f"Laser: No valid data, using estimate position_x")
                LASER_data_flag = 0
            if len(laser_y) == 0:
                # laser_y.append(self.est_pos[1])
                self.real.get_logger().info(f"Laser: No valid data, using estimate position_y")
                LASER_data_flag = 0

            final_x = np.mean(laser_x)
            final_y = np.mean(laser_y)

            # if data == [-1] * len(LASER_ANGLES):
            #     LASER_data_flag = 1
            # else:
            #     LASER_data_flag = 1
            
            # self.real.get_logger().info(f"Laser: flag: {flag}, data: {data}, up_width: {up_width}, down_width: {down_width}, estimate: ({final_x:.4f}, {final_y:.4f}), angle: {laser_est_yaw:.4f}")
            self.real.get_logger().info(f"Laser: position after solving the equation(x: {final_x:.4f}, y: {final_y:.4f}), laser_est_yaw: {laser_est_yaw:.4f}")

            return [final_x, final_y], laser_est_yaw

    

        def update(self):
            global LASER_data_flag
            # self.estrobot.update_imu()

            # Get the information from the laser
            laser_position, laser_position_angle = self.update_laser()
            laser_state = [laser_position[0], laser_position[1], laser_position_angle]
            # Get the information from the 3D laser
            three_D_laser_position = self.real.odo_position
            three_D_laser_position_angle = self.real.yaw
            three_D_state = [three_D_laser_position[0], three_D_laser_position[1], three_D_laser_position_angle]
            


            if LASER_data_flag == 1:
                # clean the flag
                
                # Calculate the average position and angle
                for i in range (len(laser_position)):
                    weight_laser = self.real.three_D_noise_std[i] / (self.real.three_D_noise_std[i] + self.real.measurement_noise_std[i])
                    weight_3D_laser = self.real.measurement_noise_std[i] / (self.real.three_D_noise_std[i] + self.real.measurement_noise_std[i])
                    self.final_state[i] =laser_state[i] * weight_laser + three_D_state[i] * weight_3D_laser 
                self.real.get_logger().info(f"Using laser data: {self.final_state}")
                LASER_data_flag = 0        
                return self.final_state

            else:#如果没有LASER数据，使用3D laser的数据进行遗传
                # clean the flag
                self.final_state = [self.real.odo_position[0],self.real.odo_position[1], self.real.yaw]
                self.real.get_logger().info(f"Laser: No laser data, using 3D laser data: {self.final_state}")
                return self.final_state

    def __init__(self):
        super().__init__('laser_position_node')
        #recieve 激光雷达数据
        self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.scan_callback,
            10)
        
        self.real_laser=[-1] * len(LASER_ANGLES)

        # resieve 3D 雷达数据
        self.create_subscription(
            Odometry,
            'Odometry',
            self.odo_callback,
            10
        )

        # #接受IMU发送的四元数信息
        # self.create_subscription(
        #     Quaternion,
        #     'imu/quaternion',
        #     self.quat_callback,
        #     10)
        # #接受IMU发送的加速度信息
        # self.create_subscription(
        #     Vector3,
        #     'imu/acceleration',
        #     self.accel_callback,
        #     10
        # )
        # hasattr(self,"initial_angle")#判断是否有初始角度
        

        self.publisher_=self.create_publisher(PoseStamped,'/laser_position',10)#发布激光位置数据，发布到/ally/robot1/laser_position，队列长度为10，发布的数据类型为LaserPosition，LaserPosition是自定义的数据类型

        self.declare_parameters(
            namespace='',
            parameters=[
                ('FIELD_SIZE', [15.00,8.00]),
                ('three_D_noise_std', [0.1,0.1,0.05]),
                ('measurement_noise_std', [0.1,0.1,0.05]),
                ('START_POINT', [7.5,4.0]),
                ('LASER_ALLOWED_NOISE', 0.1),
                ('FREQUENCY', 1.00),
                ('DELTA_DISTANCE', 0.01),
                ('three_D_DISTANCE',0.1),
                ('GRAVITY', 9.7887)
            ]
        )
        # # 获取参数值（覆盖默认值）
        
        self.FIELD_SIZE = self.get_parameter('FIELD_SIZE').get_parameter_value().double_array_value
        self.three_D_noise_std = self.get_parameter('three_D_noise_std').get_parameter_value().double_array_value
        self.measurement_noise_std = self.get_parameter('measurement_noise_std').get_parameter_value().double_array_value
        self.START_POINT = self.get_parameter('START_POINT').get_parameter_value().double_array_value
        self.LASER_ALLOWED_NOISE = self.get_parameter('LASER_ALLOWED_NOISE').get_parameter_value().double_value
        self.FREQUENCY = self.get_parameter('FREQUENCY').get_parameter_value().double_value
        self.DELTA_DISTANCE = self.get_parameter('DELTA_DISTANCE').get_parameter_value().double_value
        self.three_D_DISTANCE = self.get_parameter('three_D_DISTANCE').get_parameter_value().double_value
        self.GRAVITY = self.get_parameter('GRAVITY').get_parameter_value().double_value

        # 打印参数值（调试用）
        self.get_logger().info(f"""
        Loaded Parameters:
        - FIELD_SIZE = {self.FIELD_SIZE}
        - three_D_noise_std = {self.three_D_noise_std}
        - measurement_noise_std = {self.measurement_noise_std}
        - START_POINT = {self.START_POINT}
        - LASER_ALLOWED_NOISE = {self.LASER_ALLOWED_NOISE}
        - FREQUENCY = {self.FREQUENCY}
        - DELTA_DISTANCE = {self.DELTA_DISTANCE}
        """)

        # 初始化 EstRobot 和 KalmanFilter
        self.est_robot = self.EstRobot(self)
        # the frequency of the timer to deliver the data    
        self.timer=self.create_timer(self.FREQUENCY,self.timer_callback)
        
    def scan_callback(self, msg):

        
        self.real_laser = np.array(msg.data)
        # self.get_logger().info(f"Real laser data: {self.real_laser}\n")
    

    def odo_callback(self, msg):
        self.odo_position=[
            msg.pose.pose.position.y + self.START_POINT[0],
            msg.pose.pose.position.x + self.START_POINT[1]
        ]
        self.odo_quaternion=[
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        #转换成欧拉角
        self.yaw = euler_from_quaternion(self.odo_quaternion)[2]

    def timer_callback(self):
        
        # 确保 statement 有效
        
        statement = self.EstRobot.update(self.est_robot)
        # 发布消息
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        #机器人坐标及姿态
        msg.pose.position.x = self.FIELD_SIZE[0] - statement[0]
        msg.pose.position.y = self.FIELD_SIZE[1] - statement[1]
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
    # est_robot = Real.EstRobot(real_node)
    # # kalman_filter = KalmanFilter(est_robot,real_node)
    # laser_position = Real.Laserposition(est_robot,real_node)
    # # 独立ROS线程
    # ros_spin_thread = threading.Thread(target=rclpy.spin, args=(real_node,))
    # ros_spin_thread.daemon = True
    # ros_spin_thread.start()
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
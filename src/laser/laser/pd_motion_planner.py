#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

class PDLocalPlanner(Node):
    def __init__(self):
        super().__init__("pd_local_planner")

        self.declare_parameter("kp",2.0)
        self.declare_parameter("kd",0.1)
        self.declare_parameter("max_linear_velocity",3.0)
        self.declare_parameter("max_angular_velocity",1.0)
        self.declare_parameter("angle",math.pi/3)

        self.kp = self.get_parameter("kp").value
        self.kd = self.get_parameter("kd").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value
        self.angle = self.get_parameter("angle").value

        self.goal_pose_sub = self.create_subscription(PoseStamped,"/goal_pose", self.goal_callback,10)
        self.pose_sub = self.create_subscription(PoseStamped,"/laser_position",self.pose_callback,10)
        self.cmd_pub = self.create_publisher(Twist,"/cmd_vel",10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1,self.control_loop)

        self.pre_dx = 0.0 
        self.pre_dy = 0.0 
        self.pre_dyaw = 0.0
        self.last_time_cycle = self.get_clock().now()
        self.current_pose = None
        self.goal_pose = None
        self.used_count = 3


    def goal_callback(self,goal_pose:PoseStamped):
        self.goal_pose = goal_pose
    def pose_callback(self, pose:PoseStamped):
        self.current_pose = pose
        self.used_count = 3
    def control_loop(self):
        cmd_vel = Twist()
        if (not self.current_pose) or (not self.goal_pose) or self.used_count<0:
            self.get_logger().info(f"Not receive current pose or goal pose")
            self.cmd_pub(cmd_vel)
            return
        else:
            self.get_logger().info(f"current_pose is (x:{self.current_pose.pose.position.x:.2f},y:{self.current_pose.pose.position.y:.2f}),goal_pose is x:{self.goal_pose.pose.position.x:.2f}y:{self.goal_pose.pose.position.y:.2f})")
            self.used_count-=1

        target_x = self.goal_pose.pose.position.x
        target_y = self.goal_pose.pose.position.y
        target_yaw = euler_from_quaternion(self.goal_pose.pose.orientation)[2]
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        yaw = euler_from_quaternion(self.current_pose.pose.orientation)[2]

        dx = target_x - x
        dy = target_y - y
        dyaw = target_yaw - yaw

        distance = math.sqrt(dx*dx+dy*dy)
        abs_dyaw = math.abs(dyaw)
        if distance <=0.05 and abs_dyaw <= 0.05:
            self.get_logger().info("Goal Received")
            return

        dt = (self.get_clock().now()-self.last_time_cycle).nanoseconds*1e-9
        dx_derivative = (dx - self.pre_dx)/dt
        dy_derivative = (dy - self.pre_dy)/dt
        dyaw_derivative = (dyaw - self.pre_dyaw)/dt

        self.last_time_cycle = self.get_clock().now()
        self.pre_dyaw = dyaw
        self.pre_dx = dx
        self.pre_dy = dy

        fake_x = max(-self.max_linear_velocity,
                             min(self.kp*dx+self.kd*dx_derivative,self.max_linear_velocity))
        fake_y = max(-self.max_linear_velocity,
                             min(self.kp*dy+self.kd*dy_derivative,self.max_linear_velocity))
        cmd_vel.angular.x = max(-self.max_angular_velocity,
                             min(self.kp*dyaw+self.kd*dyaw_derivative,self.max_angular_velocity))
        cmd_vel.linear.x,cmd_vel.linear.y = self.transform_cmd(fake_x,fake_y)
        self.cmd_pub.publish(cmd_vel)
    
    def transform_cmd(self,x,y):
        remap_x = -x*math.sin(self.angle)-y*math.cos(self.angle)
        remap_y = -x*math.cos(self.angle)+y*math.sin(self.angle)
        return remap_x,remap_y

def main():
    rclpy.init()
    pd_local_node = PDLocalPlanner()
    rclpy.spin(pd_local_node)
    rclpy.shutdown()

if __name__=="__main__":
    main()

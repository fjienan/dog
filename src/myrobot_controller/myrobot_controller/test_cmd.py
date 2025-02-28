#!/usr/bin/python3
import sys
import termios
import tty

import rclpy
# from rmoss_interfaces.msg import ChassisCmd
from geometry_msgs.msg import Twist

msg = """
This node takes keypresses from the keyboard and publishes them
as ChassisCmd messages.
---------------------------
Moving around:
        w    
   a    s    d
control mode: z for velocity, x for follow gimbal
turn : '[' for left  ']' for right
stop : space key
---------------------------
CTRL-C to quit
"""

def getKey(settings):
    # 将标准输入设置为原始模式，以便能够读取单个字符而不需要按回车键
    tty.setraw(sys.stdin.fileno())
    # 从标准输入读取一个字符
    # sys.stdin.read(1) 返回一个字符串，因为在 Linux 系统上，输入是以字符串的形式返回的
    key = sys.stdin.read(1)
    # 恢复标准输入的原始设置，使其恢复正常操作
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # 返回读取到的字符
    return key


def getChassisContolMsg(x,y,w,type = 1):
    control_info = Twist()
    control_info.linear.x = x
    control_info.linear.y = y
    control_info.linear.z = 0.0
    control_info.angular.x = 0.0
    control_info.angular.y = 0.0
    control_info.angular.z = w
    return control_info

def main():

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('control_chassis_test')
    #get params
    node.declare_parameter('v',1.0)
    node.declare_parameter('w',1.0)
    v=node.get_parameter('v').value
    w=node.get_parameter('w').value
    pub = node.create_publisher(Twist, 'chassis_cmd', 10)
    print("node params v:%f,w:%f"%(v,w))
    print(msg)
    vel_x=vel_y=vel_w=0.0
    chassis_type = 1
    while True:
        key=getKey(settings)
        if key == 'w':
            vel_x=1.0 * v
        elif key == 's':
            vel_x=-1.0 * v
        elif key == 'a':
            vel_y=1.0 * v
        elif key == 'd':
            vel_y=-1.0 * v
        elif key == '[':
            vel_w=1.0 * w
        elif key == ']':
            vel_w=-1.0 * w
        elif key == ' ':
            vel_x=vel_y=vel_w=0.0
        elif key == 'z':
            chassis_type = 1
        elif key == 'x':
            chassis_type = 2
        elif key == '\x03':
            break
        info=getChassisContolMsg(vel_x,vel_y,vel_w,chassis_type)
        pub.publish(info)

if __name__ == '__main__':
    main()
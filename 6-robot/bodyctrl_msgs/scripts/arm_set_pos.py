#!/usr/bin/env python3
import rospy
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
from std_msgs.msg import Header
# utils/motor_units.py
import math

def rad_to_deg(rad):
    return rad * 180 / math.pi

def rpm_to_radps(rpm):
    return rpm * 2 * math.pi / 60

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def send_arm_position():
    pub = rospy.Publisher('/arm/set_pos', CmdSetMotorPosition, queue_size=10)
    rospy.init_node('arm_position_commander', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        # 创建命令消息
        cmd_msg = CmdSetMotorPosition()
        cmd_msg.header = Header(stamp=rospy.Time.now(), frame_id="arm_base")
        
        # 添加多个关节位置命令
        # 左臂：11---17
        cmd1 = SetMotorPosition(name=14, 
                               pos=-0.1,   # 弧度
                               spd=100,    # RPM
                               cur=50)    # 安培
        # 右臂：21---27；
        cmd2 = SetMotorPosition(name=24, 
                               pos=-0.1,   # 弧度
                               spd=100, 
                               cur=50)
        
        cmd_msg.cmds = [cmd1, cmd2]
        
        # 发布消息
        pub.publish(cmd_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_arm_position()
    except rospy.ROSInterruptException:
        pass
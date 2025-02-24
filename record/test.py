#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # 初始化rospy节点
    rospy.init_node('move_robot', anonymous=True)

    # 创建发布者，发布到/cmd_vel话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # 创建Twist消息
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # 平移速度（m/s）
        twist_msg.angular.z = 0.2  # 旋转速度（rad/s）

        # 发布消息
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
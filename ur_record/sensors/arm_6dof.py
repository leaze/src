#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   arm_6dof.py
@Time    :   2025/06/28 19:50:05
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from geometry_msgs.msg import WrenchStamped
import rospy


class Arm6Dof():
    def __init__(self):
        self.arm_6dof_left_status = [0.] * 6
        self.arm_6dof_right_status = [0.] * 6
        self.arm_6dof_left_sub = rospy.Subscriber("/arm_6dof_left", WrenchStamped, self.arm_6dof_left_callback)
        self.arm_6dof_right_sub = rospy.Subscriber("/arm_6dof_right", WrenchStamped, self.arm_6dof_right_callback)
        self.arm_6dof_timer = rospy.Timer(rospy.Duration(1), self.print_status_callback)
        rospy.sleep(0.1)

    def arm_6dof_left_callback(self, msg: WrenchStamped):
        self.arm_6dof_left_status = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        # rospy.loginfo("arm_6dof_left_status: %s", str(self.arm_6dof_left_status))
    def print_status_callback(self, event):
        """定时器回调函数，打印当前状态"""
        # rospy.loginfo("arm_6dof_left_status: %s", str(self.arm_6dof_left_status))
        # rospy.loginfo("arm_6dof_left_status: %s", str(self.arm_6dof_right_status))
        pass
    def arm_6dof_right_callback(self, msg: WrenchStamped):
        self.arm_6dof_right_status = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        # rospy.loginfo("arm_6dof_right_status: %s", str(self.arm_6dof_right_status))

if __name__ == "__main__":
    rospy.init_node("Arm6Dof")
    arm_6_dof = Arm6Dof()
    rospy.sleep(0.1)
    rospy.spin()

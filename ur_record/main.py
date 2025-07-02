#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   main.py
@Time    :   2025/06/28 19:43:09
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from controllers.arm_controller import ArmController
from mmodules.inspire_controller import InspireController
from sensors.arm_6dof import Arm6Dof
import rospy

class RobotController:
    def __init__(self):
        self.arm_controller = ArmController()
        self.hand_controller = InspireController()
        self.arm_6dof = Arm6Dof()

if __name__ == "__main__":
    rospy.init_node("RobotControllerNode")
    robot_controller = RobotController()
    robot_controller.arm_controller.run()
    left_grip_success, right_grip_success = robot_controller.hand_controller.grip_both_hands()
    rospy.loginfo(f"Left Hand Grip Success: {left_grip_success}, Right Hand Grip Success: {right_grip_success}")
    rospy.loginfo("Robot Controller is running...")
    rospy.sleep(0.1)
    rospy.spin()
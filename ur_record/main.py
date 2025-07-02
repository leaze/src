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
        rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("RobotControllerNode")
    robot_controller = RobotController()
    arm_init_success = robot_controller.arm_controller.init_arm_status()
    rospy.loginfo(f"Arm Initialization Status: {arm_init_success}")
    hand_init_success = robot_controller.hand_controller.init_hand_status()
    rospy.loginfo(f"Hand Initialization Status: {hand_init_success}")
    move_success = robot_controller.arm_controller.control_loop()
    rospy.loginfo(f"Arm Control Loop Status: {move_success}")
    if move_success:
        left_grip_success, right_grip_success = robot_controller.hand_controller.grip_both_hands()
        rospy.loginfo(f"Left Hand Grip Status: {left_grip_success}, Right Hand Grip Status: {right_grip_success}")
        if left_grip_success and right_grip_success:
            down_success = robot_controller.arm_controller.move_up(0.05)
            rospy.loginfo(f"Arm Move Up Status: {down_success}")
            if down_success:
                # 抓住楔子移动到纸箱前准备插入
                pass


    rospy.spin()
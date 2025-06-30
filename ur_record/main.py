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
from sensors.arm_6dof import Arm6Dof
import rospy

class RobotController:
    def __init__(self):
        self.arm_controller = ArmController()
        self.arm_6dof = Arm6Dof()

if __name__ == "__main__":
    robot_controller = RobotController()
    robot_controller.arm_controller.run()
    rospy.sleep(0.1)
    rospy.spin()
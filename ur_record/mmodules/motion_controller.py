#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   motion_controller.py
@Time    :   2025/07/15 13:08:29
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from hric_msgs.msg import MotionStatus
from motion_mode import MotionMode
import rospy
import time


class motion_controller:
    def __init__(self):
        rospy.init_node('motion_controller', anonymous=True)
        self.motion_mode = MotionMode()

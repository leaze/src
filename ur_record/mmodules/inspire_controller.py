#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   inspire_controller.py
@Time    :   2025/07/01 21:07:15
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from mmodules.inspire_hand import InspireHandController
import numpy as np
import rospy

class InspireController:
    def __init__(self):
        self.left_hand_controller = InspireHandController(is_left=True)
        self.right_hand_controller = InspireHandController(is_left=False)
        self.hand_controller = [self.right_hand_controller, self.left_hand_controller]
        self.angle_tolerance = 0.01  # 角度容忍度
        self.left_init_position = [1.0, 1.0, 1.0, 1.0, 1.0, 0.01]
        self.right_init_position = [1.0, 1.0, 1.0, 1.0, 1.0, 0.01]
        # self.init_hand_status()
        rospy.sleep(0.1)
    
    def move_single_hand(self, is_left:bool, angles:list) -> bool:
        """
        移动单只手
        :param is_left: 是否为左手
        :param angles: 手指角度列表
        :return: 是否成功
        """
        return self.hand_controller[is_left].set_angles(angles)
    def move_both_hands(self, left_angles:list, right_angles:list) -> tuple:
        """
        移动双只手
        :param left_angles: 左手手指角度列表
        :param right_angles: 右手手指角度列表
        :return: 是否成功
        """
        left_success = self.left_hand_controller.set_angles(left_angles)
        right_success = self.right_hand_controller.set_angles(right_angles)
        return left_success, right_success
    
    def grip_single_hand(self, is_left:bool) -> bool:
        """
        握住单只手
        :param is_left: 是否为左手
        :return: 是否成功
        """
        # 打印调试信息
        grip_single_hand_status = self.move_single_hand(is_left, [0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        rospy.loginfo("Single hand moved successfully") if grip_single_hand_status else rospy.logerr("Single hand moved failed")
        return grip_single_hand_status
    
    def grip_both_hands(self) -> bool:
        """
        握住双只手
        :return: 是否成功
        """
        # 打印调试信息
        grip_left_hands_status, grip_right_hands_status = self.move_both_hands([0.1, 0.1, 0.1, 0.1, 0.5, 0.5], [0.1, 0.1, 0.1, 0.1, 0.5, 0.5])
        rospy.loginfo("Both hands grip successfully") if grip_left_hands_status and grip_right_hands_status else rospy.logerr("Both hands grip failed")
        return grip_left_hands_status and grip_right_hands_status
    
    def release_single_hand(self, is_left:bool) -> bool:
        """
        松开单只手
        :param is_left: 是否为左手
        :return: 是否成功
        """
        # 打印调试信息
        release_single_hand_status = self.move_single_hand(is_left, [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        rospy.loginfo("Single hand released successfully") if release_single_hand_status else rospy.logerr("Single hand release failed")
        return release_single_hand_status
    
    def release_both_hands(self) -> bool:
        """
        松开双只手
        :return: 是否成功
        """
        # 打印调试信息
        release_left_hand_status, release_right_hand_status = self.move_both_hands([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        rospy.loginfo("Release both hands successfully") if release_left_hand_status and release_right_hand_status else rospy.logerr("Release both hands failed")
        return release_left_hand_status and release_right_hand_status
    
    def grip_wedge(self) -> bool:
        """
        握住双只手
        :return: 是否成功
        """
        # 打印调试信息
        grip_left_hands_status, grip_right_hands_status = self.move_both_hands([0.99, 0.99, 0.99, 0.99, 0.01, 0.01], [0.99, 0.99, 0.99, 0.99, 0.01, 0.01])
        rospy.sleep(1.5)
        rospy.loginfo("Both hands grip wedge successfully") if grip_left_hands_status and grip_right_hands_status else rospy.logerr("Both hands grip wedge failed")
        return grip_left_hands_status and grip_right_hands_status

    def release_wedge(self) -> bool:
        """
        松开楔子
        :return: 是否成功
        """
        # 打印调试信息
        release_left_hands_status, release_right_hands_status = self.move_both_hands([1.0, 1.0, 1.0, 1.0, 1.0, 0.01], [1.0, 1.0, 1.0, 1.0, 1.0, 0.01])
        rospy.sleep(1.5)
        rospy.loginfo("Both hands release wedge successfully") if release_left_hands_status and release_right_hands_status else rospy.logerr("Both hands release wedge failed")
        return release_left_hands_status and release_right_hands_status

    def grip_box(self) -> bool:
        """
        握住箱子
        :return: 是否成功
        """
        # 打印调试信息
        grip_left_hands_status, grip_right_hands_status = self.move_both_hands([0.01, 0.01, 0.01, 0.01, 0.1, 0.01], [0.01, 0.01, 0.01, 0.01, 0.1, 0.01])
        rospy.sleep(1.5)
        rospy.loginfo("Both hands grip box successfully") if grip_left_hands_status and grip_right_hands_status else rospy.logerr("Both hands grip box failed")
        return grip_left_hands_status and grip_right_hands_status

    def release_wedge(self) -> bool:
        """
        松开箱子
        :return: 是否成功
        """
        # 打印调试信息
        release_left_hands_status, release_right_hands_status = self.move_both_hands([1.0, 1.0, 1.0, 1.0, 1.0, 0.01], [1.0, 1.0, 1.0, 1.0, 1.0, 0.01])
        rospy.sleep(1.5)
        rospy.loginfo("Both hands release box successfully") if release_left_hands_status and release_right_hands_status else rospy.logerr("Both hands release box failed")
        return release_left_hands_status and release_right_hands_status


    def init_hand_status(self):
        """
        初始化手的状态
        :return: None
        """
        left_success = self.left_hand_controller.set_angles(self.left_init_position)
        right_success = self.right_hand_controller.set_angles(self.right_init_position)
        rospy.loginfo("Inspire Hand Controller Initializing...")
        rospy.sleep(2)
        if self.left_hand_controller.position is None or self.right_hand_controller.position is None:
            rospy.logerr("Hand position not initialized properly")
            return False
        else:
            left_hand_error_ = np.linalg.norm(np.array(self.left_hand_controller.position) - np.array(self.left_init_position))
            right_hand_error_ = np.linalg.norm(np.array(self.right_hand_controller.position) - np.array(self.right_init_position))
            hand_error_ = (left_hand_error_ + right_hand_error_) / 2.0
            rospy.loginfo(f"Left Hand Status: {left_hand_error_}, Right Hand Status: {right_hand_error_}")
            return hand_error_ < self.angle_tolerance

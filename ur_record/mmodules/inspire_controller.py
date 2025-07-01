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
from inspire_hand import InspireHandController
import rospy

class InspireController:
    def __init__(self):
        self.left_hand_controller = InspireHandController(is_left=True)
        self.right_hand_controller = InspireHandController(is_left=False)
        self.hand_controller = [self.right_hand_controller, self.left_hand_controller]
    
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
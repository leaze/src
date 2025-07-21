#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""
@File    :   main.py
@Time    :   2025/06/28 19:43:09
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
"""
from controllers.arm_controller import ArmController
from mmodules.inspire_controller import InspireController
from sensors.arm_6dof import Arm6Dof
import numpy as np
import rospy
import math


class RobotController:
    def __init__(self):
        self.arm_controller = ArmController()
        self.hand_controller = InspireController()
        self.arm_6dof = Arm6Dof()
        self.current_state = None
        self.left_init_joints = [0.0, 0.15, 0.0, -0.0, 0.0, 0.0, -0.0]
        self.right_init_joints = [0.0, -0.15, 0.0, -0.0, 0.0, 0.0, -0.0]
        rospy.sleep(0.1)

    def recveive_command(self):
        # 接收指令
        left_xyz_ = [0.3274565150820438, 0.19676078988506351, -0.07188115117621655]
        left_quat_ = [0.6549714783818681, -0.5350813541603976, -0.36645522080454546, 0.3878284131948522]
        right_xyz_ = [0.3274565150820438, -0.19676078988506351, -0.07188115117621655]
        right_quat_ = [0.6549714783818681, 0.5350813541603976, -0.36645522080454546, -0.3878284131948522]
        return left_xyz_, left_quat_, right_xyz_, right_quat_

    def location_wedge(self, left_xyz_=None, left_quat_=None, right_xyz_=None, right_quat_=None):
        # 移动各个关节轴
        left_position, right_position = [0.0, 0.15, 0.0, -1.0, 0.0, 0.0, -0.0], [0.0, -0.15, 0.0, -1.0, -0.0, 0.0, 0.0]
        self.arm_controller.send_arms_cmd_pos_service(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        # rospy.sleep(2.0)
        dual_joint_error_ = np.linalg.norm(np.array(self.arm_controller.left_joint_positions + self.arm_controller.right_joint_positions) - np.array(left_position + right_position))
        rospy.loginfo(f"Arm Location Step1 Status: {dual_joint_error_ < self.arm_controller.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")
        # 定位楔子
        left_target_pos_ = [0.3274565150820438, 0.19676078988506351, -0.07188115117621655]
        left_target_quat_ = [0.6549714783818681, -0.5350813541603976, -0.36645522080454546, 0.3878284131948522]

        right_target_pos_ = [0.3274565150820438, -0.19676078988506351, -0.07188115117621655]
        right_target_quat_ = [0.6549714783818681, 0.5350813541603976, -0.36645522080454546, -0.3878284131948522]
        # 发送消息
        move_wedge_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        grab_wedge_status = self.hand_controller.grip_wedge()
        return move_wedge_success and grab_wedge_status

    def grab_wedge(self):
        # 抓取楔子
        grab_wedge_status = self.hand_controller.grip_wedge()
        rospy.sleep(1)
        grab_box_status = self.hand_controller.grip_box()
        return grab_wedge_status and grab_box_status

    def lift_wedge(self):
        left_rotate_angle_ = [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        right_rotate_angle_ = [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rotate_success = self.arm_controller.rotate_joint(left_rotate_angle_, right_rotate_angle_)
        return rotate_success

    def move_wedge(self, left_xyz_=None, left_quat_=None, right_xyz_=None, right_quat_=None):
        left_start_pos_ = self.arm_controller.left_end_effector_pose
        left_start_quat_ = self.arm_controller.left_end_effector_quat
        right_start_pos_ = self.arm_controller.right_end_effector_pose
        left_start_quat_ = self.arm_controller.right_end_effector_quat
        # 左手移动到纸箱前
        left_target_pos_ = [0.10371069, left_start_pos_[1], -0.01846677]
        left_target_quat_ = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]
        # 右手移动到纸箱前
        right_target_pos_ = [0.10371069, right_start_pos_[1], -0.01846677]
        right_target_quat_ = [0.645778878918554, 0.5312803574078042, -0.37486812155046495, -0.40023082442580193]
        # 发送消息
        move_wedge_success = self.arm_controller.move_dual_arm_by_xyz_tr(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        return move_wedge_success

    def insert_wedge(self):
        # 插入楔子
        left_target_pos_ = [0.32371069, self.arm_controller.left_end_effector_pose[1], -0.06846677]
        left_target_quat_ = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]

        right_target_pos_ = [0.32371069, self.arm_controller.right_end_effector_pose[1], -0.01846677]
        right_target_quat_ = [0.645778878918554, 0.5312803574078042, -0.37486812155046495, -0.40023082442580193]
        # 发送消息
        # insert_wedge_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        insert_wedge_success = self.arm_controller.move_dual_forward(0.05)
        return insert_wedge_success

    def grab_box(self):
        # 抓取纸箱
        # 1 释放楔子
        release_wedge_status = self.hand_controller.release_wedge()
        # 2 移动
        move_left_status = self.arm_controller.rotate_joint([0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        # 3 移动到抓取点
        # 3.1 左手移动到纸箱抓取点
        left_target_pos_ = [0.12371069, 0.21492773, -0.06846677]
        left_target_quat_ = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]
        # 3.2 右手移动到纸箱抓取点
        right_target_pos_ = [0.12371069, -0.21492773, -0.06846677]
        right_target_quat_ = [0.645778878918554, 0.5312803574078042, -0.37486812155046495, -0.40023082442580193]
        # 3.3 发送消息
        move_to_box_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # 4 抓取箱子
        grab_left_box_status, grab_right_box_status = self.hand_controller.grip_box()
        return release_wedge_status and move_left_status and move_to_box_success and grab_left_box_status and grab_right_box_status

    def lift_box(self, distance):
        # 向上移动的逻辑
        return self.arm_controller.move_dual_up(distance)

    def move_box(self):
        # 移动纸箱的逻辑
        move_box_status = self.arm_controller.move_dual_down(0.05)

        return move_box_status

    def place_box(self):
        # 置纸箱的逻辑
        pass

    def get_arm_state(self):
        print("left_joint_positions = ", self.arm_controller.left_joint_positions)
        print("left_end_effector_pose = ", self.arm_controller.left_end_effector_pose)
        print("left_end_effector_quat = ", self.arm_controller.left_end_effector_quat)
        print("right_joint_positions = ", self.arm_controller.right_joint_positions)
        print("right_end_effector_pose = ", self.arm_controller.right_end_effector_pose)
        print("right_end_effector_quat = ", self.arm_controller.right_end_effector_quat)

    def test(self):
        # 抓取楔子
        self.grab_wedge()
        # 抬起楔子
        self.lift_wedge()
        # 移动到纸箱前
        self.move_wedge()
        # 插入楔子
        self.insert_wedge()

if __name__ == "__main__":
    rospy.init_node("RobotControllerNode")
    robot_controller = RobotController()
    # robot_controller.get_arm_state()
    hand_init_success = robot_controller.hand_controller.init_hand_status()
    arm_init_success = robot_controller.arm_controller.init_arm_status()
    robot_controller.move_wedge()
    print("Service test end")
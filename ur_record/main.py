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
        pass

    def location_wedge(self):
        # 移动各个关节轴
        left_position, right_position = [0.0, 0.15, 0.0, -1.0, 0.0, 0.0, -0.0], [0.0, -0.15, 0.0, -1.0, -0.0, 0.0, 0.0]
        self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        rospy.sleep(2.0)
        dual_joint_error_ = np.linalg.norm(np.array(self.arm_controller.left_joint_positions + self.arm_controller.right_joint_positions) - np.array(left_position + right_position))
        rospy.loginfo(f"Arm Location Step1 Status: {dual_joint_error_ < self.arm_controller.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")
        # 定位楔子
        left_target_pos_ = [0.32371069, 0.20492773, -0.06846677]
        left_target_quat_ = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]
        left_start_pos_ = self.arm_controller.left_joint_positions
        left_target_joint_ = self.arm_controller.arm_kinematics[True].inverse_kinematics(left_target_pos_, left_target_quat_, left_start_pos_)
        
        right_target_pos_ = [0.32371069, -0.20492773, -0.06846677]
        right_target_quat_ = [0.645778878918554, 0.5312803574078042, -0.37486812155046495, -0.40023082442580193]
        right_start_pos_ = self.arm_controller.right_joint_positions
        right_target_joint_ = self.arm_controller.arm_kinematics[False].inverse_kinematics(right_target_pos_, right_target_quat_, right_start_pos_)
        rospy.loginfo("Location Wedge suceessed")
        self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], list(left_target_joint_) + list(right_target_joint_), [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        rospy.sleep(2.0)
        
        dual_joint_error_ = np.linalg.norm(np.array(self.arm_controller.left_joint_positions + self.arm_controller.right_joint_positions) - np.array(list(left_target_joint_) + list(right_target_joint_)))
        rospy.loginfo(f"Arm Location Step1 Status: {dual_joint_error_ < self.arm_controller.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")
        # 抓取楔子
        grab_left_wedge_status, grab_right_wedge_status = self.hand_controller.move_both_hands([0.8, 0.8, 0.8, 0.8, 0.1, 0.01], [0.8, 0.8, 0.8, 0.8, 0.1, 0.01])
        rospy.loginfo("Grab wedge succeeded") if grab_left_wedge_status and grab_right_wedge_status else rospy.logerr("Grab wedge failed")
        rospy.sleep(1)

    def grab_wedge(self):
        # 移动各个关节轴
        left_position, right_position = [0.0, 0.15, 0.0, -1.0, 0.0, 0.0, -0.0], [0.0, -0.15, 0.0, -1.0, -0.0, 0.0, 0.0]
        grab_step1_success = self.arm_controller.rotate_dual_joint(left_position, right_position)
        # 定位楔子
        left_position = [-0.4157094955444336, -0.15037822723388672, 0.2577095031738281, -0.48166990280151367, 0.9909520149230957, 0.3322734832763672, -0.610745906829834]
        right_position = [-0.4157094955444336, 0.15037822723388672, -0.2577095031738281, -0.48166990280151367, -0.9909520149230957, 0.3322734832763672, 0.610745906829834]
        grab_step2_success = self.arm_controller.rotate_dual_joint(left_position, right_position)
        # 抓取楔子
        grab_wedge_status = self.hand_controller.grip_wedge()
        return grab_step1_success and grab_step2_success and grab_wedge_status

    def lift_wedge(self):
        # # 抬起楔子: 1 给定固定各个关节角度
        # 移动各个关节轴
        left_position = [-0.7157459259033203, -0.15037822723388672, 0.2577095031738281, -0.48166990280151367, 0.9909520149230957, 0.3322734832763672, -0.610745906829834]
        right_position = [-0.7157459259033203, 0.15040206909179688, -0.2577328681945801, -0.4817180633544922, -0.9909753799438477, 0.33171796798706055, 0.610295295715332]
        dual_rotate_success = self.arm_controller.rotate_dual_joint(left_position, right_position)
        return dual_rotate_success

    def move_wedge(self):
        # 左手移动到纸箱前
        left_target_pos_ = [0.15371069, 0.20492773, -0.01846677]
        left_target_quat_ = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]
        # 右手移动到纸箱前
        right_target_pos_ = [0.15371069, -0.20492773, -0.01846677]
        right_target_quat_ = [0.645778878918554, 0.5312803574078042, -0.37486812155046495, -0.40023082442580193]
        # 发送消息
        move_wedge_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        return move_wedge_success

    def insert_wedge(self):
        # 插入楔子
        inser_wedge_status = self.arm_controller.move_dual_forward(0.05)
        rospy.loginfo("Move forward wedge succeeded") if inser_wedge_status else rospy.logerr("Move forward wedge failed")
        return inser_wedge_status

    def grab_box(self):
        # 抓取纸箱
        # 1 释放楔子
        release_wedge_status = self.hand_controller.release_wedge()
        # 2 移动
        move_left_status = self.arm_controller.rotate_joint([0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        # 3 移动到抓取点
        # 3.1 左手移动到纸箱抓取点
        left_target_pos_ = [0.15371069, 0.20492773, -0.06846677]
        left_target_quat_ = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]
        # 3.2 右手移动到纸箱抓取点
        right_target_pos_ = [0.15371069, -0.20492773, -0.06846677]
        right_target_quat_ = [0.645778878918554, 0.5312803574078042, -0.37486812155046495, -0.40023082442580193]
        # 3.3 发送消息
        move_to_box_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # 4 抓取箱子
        grab_left_box_status, grab_right_box_status = self.hand_controller.grip_box()
        return release_wedge_status and move_left_status and move_to_box_success and grab_left_box_status and grab_right_box_status

    def move_box(self):
        # 移动纸箱的逻辑
        move_box_status = self.arm_controller.move_dual_down(0.01)
        return move_box_status

    def place_box(self):
        # 置纸箱的逻辑
        pass

    def lift_box(self, distance):
        # 向上移动的逻辑
        return self.arm_controller.move_dual_up(distance)

    def get_arm_state(self):
        print("left_joint_positions = ", self.arm_controller.left_joint_positions)
        print("left_end_effector_pose = ", self.arm_controller.left_end_effector_pose)
        print("left_end_effector_quat = ", self.arm_controller.left_end_effector_quat)
        print("right_joint_positions = ", self.arm_controller.right_joint_positions)
        print("right_end_effector_pose = ", self.arm_controller.right_end_effector_pose)
        print("right_end_effector_quat = ", self.arm_controller.right_end_effector_quat)

    def run(self):
        grab_wedge_success = self.grab_wedge() if hand_init_success and arm_init_success else False
        lift_wedge_success = self.lift_wedge() if grab_wedge_success else False
        move_wedge_success = self.move_wedge() if lift_wedge_success else False
        insert_wedge_success = self.insert_wedge() if move_wedge_success else False

    def test(self):
        # 移动各个关节轴
        print("Service test start")
        left_position = [-0.712834154641907, -0.03955463815729167, 0.4867647932568923, 0.2026527607047936, 0.7588279682947596, 0.09691369817309022, -0.9072397551933101]
        right_position = [-0.712834154641907, 0.03955463815729167, -0.4867647932568923, 0.2026527607047936, -0.7588279682947596, 0.09691369817309022, 0.9072397551933101]
        right_position = self.right_init_joints
        self.arm_controller.send_arms_cmd_pos_service(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        self.hand_controller.grip_wedge()
        self.lift_wedge()
        left_position = [0.18982091844311913, 0.33477392974885584, -0.645854768961989, -1.5611050390534627, 1.8114655917507176, -0.18542333917518908, -0.24889762080904546]
        right_position = [0.18982091844311913, -0.33477392974885584, 0.645854768961989, -1.5611050390534627, -1.8114655917507176, -0.18542333917518908, 0.24889762080904546]
        right_position = self.right_init_joints
        self.arm_controller.send_arms_cmd_pos_service(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        print("Service test end")


if __name__ == "__main__":
    rospy.init_node("RobotControllerNode")
    robot_controller = RobotController()
    robot_controller.get_arm_state()
    hand_init_success = robot_controller.hand_controller.init_hand_status()
    arm_init_success = robot_controller.arm_controller.init_arm_status()
    # robot_controller.grab_wedge()
    # robot_controller.move_wedge()
    # robot_controller.test()
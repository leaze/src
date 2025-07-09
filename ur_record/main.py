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

def deg2rad(deg_ls: list):
    return [round(math.radians(deg), 3) for deg in deg_ls]

class RobotController:
    def __init__(self):
        self.arm_controller = ArmController()
        self.hand_controller = InspireController()
        self.arm_6dof = Arm6Dof()
        self.current_state = None
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
        right_target_pos_ = [0.32458236, -0.11052364, -0.07044139]
        right_target_quat_ = [0.615048141451942, 0.5939422367756204, -0.32283101752874543, -0.4058676350632553]
        right_start_pos_ = self.arm_controller.right_joint_positions
        right_target_joint_ = self.arm_controller.arm_kinematics[False].inverse_kinematics(right_target_pos_, right_target_quat_, right_start_pos_)
        rospy.loginfo("Location Wedge suceessed")
        self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], list(left_target_joint_) + list(right_target_joint_), [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        rospy.sleep(2.0)
        dual_joint_error_ = np.linalg.norm(np.array(self.arm_controller.left_joint_positions + self.arm_controller.right_joint_positions) - np.array(list(left_target_joint_) + list(right_target_joint_)))
        rospy.loginfo(f"Arm Location Step1 Status: {dual_joint_error_ < self.arm_controller.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")

    def grab_wedge(self):
        # 移动各个关节轴
        left_position, right_position = [0.0, 0.15, 0.0, -1.0, 0.0, 0.0, -0.0], [0.0, -0.15, 0.0, -1.0, -0.0, 0.0, 0.0]
        self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        rospy.sleep(2.0)
        dual_joint_error_ = np.linalg.norm(np.array(self.arm_controller.left_joint_positions + self.arm_controller.right_joint_positions) - np.array(left_position + right_position))
        rospy.loginfo(f"Arm Location Step1 Status: {dual_joint_error_ < self.arm_controller.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")
        # 定位楔子
        left_position = [-0.4157094955444336, -0.15037822723388672, 0.2577095031738281, -0.48166990280151367, 0.9909520149230957, 0.3322734832763672, -0.610745906829834]
        right_position = [-0.4157094955444336, 0.15037822723388672, -0.2577095031738281, -0.48166990280151367, -.9909520149230957, 0.3322734832763672, 0.610745906829834]
        self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        rospy.sleep(2.0)
        dual_joint_error_ = np.linalg.norm(np.array(self.arm_controller.left_joint_positions + self.arm_controller.right_joint_positions) - np.array(left_position + right_position))
        rospy.loginfo(f"Arm Location Wedge Status: {dual_joint_error_ < self.arm_controller.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")
        # 抓取楔子
        grab_left_wedge_status, grab_right_wedge_status = self.hand_controller.move_both_hands([1.0, 1.0, 1.0, 1.0, 0.1, 0.01], [1.0, 1.0, 1.0, 1.0, 0.1, 0.01])
        rospy.loginfo("Grab wedge succeeded") if grab_left_wedge_status and grab_right_wedge_status else rospy.logerr("Grab wedge failed")
        # 抬起楔子
        # move_up_wedge_status = self.arm_controller.move_dual_up(0.05)
        # rospy.loginfo("Move up wedge succeeded") if move_up_wedge_status else rospy.logerr("Move up wedge failed")
        return 

    def move_to_box(self):
        # 移动到纸箱前
        move_to_box_backward_status = self.arm_controller.move_dual_backward(0.05)
        rospy.loginfo("Move backward wedge succeeded") if move_to_box_backward_status else rospy.logerr("Move backward wedge failed")
        move_to_box_down_status = self.arm_controller.move_dual_down(0.05)
        rospy.loginfo("Move down wedge succeeded") if move_to_box_down_status else rospy.logerr("Move down wedge failed")
        return move_to_box_backward_status and move_to_box_down_status

    def insert_wedge(self):
        # 插入楔子
        inser_wedge_status = self.arm_controller.move_dual_forward(0.05)
        rospy.loginfo("Move forward wedge succeeded") if inser_wedge_status else rospy.logerr("Move forward wedge failed")
        return inser_wedge_status

    def grab_box(self):
        # 抓取纸箱
        release_wedge_status = self.hand_controller.release_both_hands()
        rospy.loginfo("Release wedge succeeded") if release_wedge_status else rospy.logerr("Release wedge failed")
        move_left_status = self.arm_controller.move_single_left(is_left=True, distance=0.01)  # 左臂左移
        move_right_status = self.arm_controller.move_single_right(is_left=False, distance=0.01)  # 右臂右移
        move_up_status = self.arm_controller.move_dual_up(0.01)
        # 旋转腕部
        self.arm_controller.rotate_single_joint(True, 6, rotate_angle=0.0)  # 左腕旋转
        self.arm_controller.rotate_single_joint(False, 6, rotate_angle=-0.0)  # 右腕旋转
        grab_box_status = self.hand_controller.grip_both_hands()
        rospy.loginfo("Grab box succeeded") if grab_box_status else rospy.logerr("Grab box failed")
        return release_wedge_status and move_left_status and move_right_status and move_up_status and grab_box_status

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
        print(self.arm_controller.left_joint_positions)
        print(self.arm_controller.left_end_effector_pose)
        print(self.arm_controller.left_end_effector_quat)
        print(self.arm_controller.right_joint_positions)
        print(self.arm_controller.right_end_effector_pose)
        print(self.arm_controller.right_end_effector_quat)

    def test(self):
        # # 移动各个关节轴
        # left_ls, right_ls = [-45, 10, -90, -30, 0, 0, 0], [-45, -10, 90, -30, 0, 0, 0]
        # left_position, right_position = [-0.6780195236206055, -0.1549086570739746, 0.3328738212585449, -0.29601097106933594, 0.6902914047241211, 0.3504042625427246, -0.4729442596435547], [-0.4019031524658203, 0.11811637878417969, -0.43258237838745117, -0.6672816276550293, -0.8666987419128418, 0.2319793701171875, 0.32726430892944336]
        # self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        # rospy.sleep(2.0)

        # left_ls, right_ls = [-60, 10, -90, -30, 0, 0, 0], [-60, -10, 90, -30, 0, 0, 0]
        # left_position, right_position = deg2rad(left_ls), deg2rad(right_ls)
        # self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        # rospy.sleep(2.0)
        
        # # 旋转腕部
        # self.arm_controller.rotate_single_joint(True, 1, rotate_angle=0.15)  # 左腕旋转
        # self.arm_controller.rotate_single_joint(False, 2, rotate_angle=-0.15)  # 右腕旋转

        # 灵巧手
        self.hand_controller.move_both_hands([1.0, 1.0, 1.0, 1.0, 0.1, 0.01], [1.0, 1.0, 1.0, 1.0, 0.1, 0.01])
        # self.arm_controller.move_dual_up(0.05)


if __name__ == "__main__":
    rospy.init_node("RobotControllerNode")
    robot_controller = RobotController()
    hand_init_success = robot_controller.hand_controller.init_hand_status()
    arm_init_success = robot_controller.arm_controller.init_arm_status()
    # robot_controller.location_wedge()
    # robot_controller.test()

    rospy.spin()

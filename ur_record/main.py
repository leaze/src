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
from sensors.orrb_camera import OrrbCamera
from sensors.arm_6dof import Arm6Dof
from tools.trans import Trans
import numpy as np
import rospy
import math


class RobotController:
    def __init__(self):
        self.arm_controller = ArmController()
        self.hand_controller = InspireController()
        self.arm_6dof = Arm6Dof()
        self.orrb_camera = OrrbCamera()
        self.trans = Trans()
        self.current_state = None
        self.left_init_joints = [0.0, 0.15, 0.0, -0.0, 0.0, 0.0, -0.0]
        self.right_init_joints = [0.0, -0.15, 0.0, -0.0, 0.0, 0.0, -0.0]
        rospy.sleep(0.1)

    def recveive_command(self):
        # 接收指令
        while True:
            if self.orrb_camera.flag_wedge:
                world_xyz_, world_quat_ = self.orrb_camera.get_current_pose()
                print(world_xyz_)
                return world_xyz_, world_quat_
        pass

    def location_wedge(self):
        # 移动各个关节轴
        left_position, right_position = [0.0, 0.15, 0.0, -1.0, 0.0, 0.0, -0.0], [0.0, -0.15, 0.0, -1.0, -0.0, 0.0, 0.0]
        self.arm_controller.send_arms_cmd_pos_service(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], left_position + right_position, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        # rospy.sleep(2.0)
        dual_joint_error_ = np.linalg.norm(np.array(self.arm_controller.left_joint_positions + self.arm_controller.right_joint_positions) - np.array(left_position + right_position))
        rospy.loginfo(f"Arm Location Step1 Status: {dual_joint_error_ < self.arm_controller.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")
        # 定位楔子
        left_pos = [0.4409711531681323, 0.21492773315409157, -0.10711783728977194]
        left_quat = [-0.5824993766583517, 0.5851301607905339, 0.4671733566939692, -0.316332461061405]

        right_pos = [0.44098031001017535, -0.21493083517530405, -0.107130417934516805]
        right_quat = [0.5825471357726449, 0.5848687698290453, -0.4673399686813965, -0.31648176938746125]
        # 发送消息
        move_wedge_success = self.arm_controller.move_dual_arm_by_xyz(left_pos, left_quat, right_pos, right_quat)
        # grab_wedge_status = self.hand_controller.grip_wedge()
        return move_wedge_success

    def grab_wedge(self):
        # 移动各个关节轴
        left_joints, right_joints = [0.0, 0.15, 0.0, -1.0, 0.0, 0.0, -0.0], [0.0, -0.15, 0.0, -1.0, -0.0, 0.0, 0.0]
        grab_step1_success = self.arm_controller.rotate_dual_joint(left_joints, right_joints)
        # 定位楔子
        left_joints = [-0.4592773565105143, -0.10781207465494458, 0.08085273463238411, -0.372475432520502, 1.1618900490820394, 0.26562390329576485, -0.6432547546848081]
        right_joints = [-0.4592773565105143, 0.10781207465494458, -0.08085273463238411, -0.372475432520502, -1.1618900490820394, 0.26562390329576485, 0.6432547546848081]
        grab_step2_success = self.arm_controller.rotate_dual_joint(left_joints, right_joints)
        # 抓取楔子
        grab_wedge_status = self.hand_controller.grip_wedge()
        rospy.sleep(1)
        grab_box_status = self.hand_controller.grip_box()
        self.arm_controller.move_dual_forward(0.1)
        return grab_step1_success and grab_step2_success and grab_wedge_status

    def lift_wedge(self):
        # # 抬起楔子: 1 给定固定各个关节角度
        # 移动各个关节轴
        # left_position = [-0.7157459259033203, -0.15037822723388672, 0.2577095031738281, -0.48166990280151367, 0.9909520149230957, 0.3322734832763672, -0.610745906829834]
        # right_position = [-0.7157459259033203, 0.15040206909179688, -0.2577328681945801, -0.4817180633544922, -0.9909753799438477, 0.33171796798706055, 0.610295295715332]
        self.arm_controller.rotate_single_joint(True, 0, rotate_angle=-0.2)
        self.arm_controller.rotate_single_joint(False, 0, rotate_angle=-0.2)
        rospy.sleep(1)
        # dual_rotate_success = self.arm_controller.rotate_dual_joint(left_position, right_position)
        # return dual_rotate_success

    def move_wedge(self):
        # 左手移动到纸箱前
        left_target_pos_ = [0.32497879, 0.19681914, -0.06855335]
        left_target_quat_ = [0.65497752, -0.53508699, -0.36644699, 0.38781821]
        # 右手移动到纸箱前
        right_target_pos_ = [0.32497879, -0.19681914, -0.06855335]
        right_target_quat_ = [0.65497752, 0.53508699, -0.36644699, -0.38781821]
        # 发送消息
        move_wedge_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # self.arm_controller.move_dual_up(0.1)
        # 左手移动到纸箱前
        left_target_pos_ = [0.15371069, 0.19029899, -0.06846677]
        left_target_quat_ = [0.65053061, -0.52448355, -0.37982903, 0.39680832]
        # 右手移动到纸箱前
        right_target_pos_ = [0.32400525, -0.19905013, 0.00859038]
        right_target_quat_ = [0.64999305, 0.52478635, -0.37940189, -0.39769653]
        # 发送消息
        move_wedge_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        return move_wedge_success

    def insert_wedge(self):
        # 插入楔子
        left_target_pos_ = [0.32404316, 0.1997382, 0.00859038]
        left_target_quat_ = [0.65053061, -0.52448355, -0.37982903, 0.39680832]

        right_target_pos_ = [0.32400525, -0.19905013, 0.00859038]
        right_target_quat_ = [0.64999305, 0.52478635, -0.37940189, -0.39769653]
        # 发送消息
        insert_wedge_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        self.hand_controller.grip_wedge()
        rospy.sleep(1)
        self.hand_controller.grip_box()
        self.arm_controller.rotate_joint([-0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [-0.25, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # 插入楔子
        left_target_pos_ = [0.15371069, 0.19029899, -0.06846677]
        left_target_quat_ = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]

        right_target_pos_ = [0.15371069, -0.19029899, -0.06846677]
        right_target_quat_ = [0.645778878918554, 0.5312803574078042, -0.37486812155046495, -0.40023082442580193]
        # # 发送消息
        insert_wedge_success = self.arm_controller.move_dual_arm_by_xyz_tr2(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_, steps=10, is_random=False, direction=[1.0, 0.0, 1.0])
        # rospy.sleep(1)
        # for i in range(5):
        #     robot_controller.arm_controller.move_dual_forward(0.05)
        return insert_wedge_success

    def grab_box(self):
        # 0 定位纸箱
        left_position, right_position = [-0.15, 0.30, 0.0, -1.0, 0.0, 0.0, -0.0], [-0.15, -0.30, 0.0, -1.0, -0.0, 0.0, 0.0]
        grab_step1_success = self.arm_controller.rotate_dual_joint(left_position, right_position)
        # 1 抓取纸箱
        left_position = [-0.8237481117248535, 0.56422786712646484, -0.23824644088745117, -0.0004076957702636719, 1.4139466285705566, 0.844813346862793, -0.0]
        right_position = [-0.8237481117248535, -0.56422786712646484, 0.23824644088745117, -0.0004076957702636719, -1.4139466285705566, 0.844813346862793, 0.0]
        grab_step2_success = self.arm_controller.rotate_dual_joint(left_position, right_position)
        # 2 移动
        left_position = [-0.8031110763549805, 0.50185794830322266, -0.5942502021789551, -0.17487382888793945, 1.866687297821045, 0.837101936340332, -0.6051468849182129]
        right_position = [-0.8031110763549805, -0.50185794830322266, 0.5942502021789551, -0.17487382888793945, -1.866687297821045, 0.837101936340332, 0.6051468849182129]
        grab_step3_success = self.arm_controller.rotate_dual_joint(left_position, right_position)

        left_position = [-0.8031110763549805, 0.35185794830322266, -0.5942502021789551, -0.17487382888793945, 1.866687297821045, 0.837101936340332, -0.6051468849182129]
        right_position = [-0.8031110763549805, -0.35185794830322266, 0.5942502021789551, -0.17487382888793945, -1.866687297821045, 0.837101936340332, 0.6051468849182129]
        grab_step3_success = self.arm_controller.rotate_dual_joint(left_position, right_position)
        self.hand_controller.grip_box()
        move_left_status = self.arm_controller.rotate_joint([-0.3, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0], [-0.3, -0.0, 0.0, -0.0, 0.0, 0.0, 0.0])

        # rospy.sleep(5)
        # move_left_status = self.arm_controller.rotate_joint([0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0])
        # 3 移动到抓取点
        # 3.1 左手移动到纸箱抓取点
        # left_target_pos_ = [0.39141683594642823, 0.3778243668227988, 0.12342496856493235]
        # left_target_quat_ = [0.6733941594331715, -0.5827220661212154, -0.24117596996335405, 0.3857582808041192]
        # # 3.2 右手移动到纸箱抓取点
        # right_target_pos_ = [0.39141683594642823, -0.3778243668227988, 0.12342496856493235]
        # right_target_quat_ = [0.6733941594331715, 0.5827220661212154, -0.24117596996335405, -0.3857582808041192]
        # self.arm_controller.move_dual_arm_by_xyz_tr(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # 3.3 发送消息
        # move_to_box_success = self.arm_controller.move_dual_arm_by_xyz(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # 4 抓取箱子
        # grab_box_status = self.hand_controller.grip_box()

    def place_box(self):
        # 置纸箱的逻辑
        left_target_pos_ = [0.15371069, 0.3863358589574437, -0.06846677]
        left_target_quat_ = [0.6394114697766845, -0.6273574282828561, -0.35111895992791475, 0.272563947497611]
        left_target_rpy_ = [-91.94376346, -6.14407546, 52.53020394]
        left_target_quat_ = self.trans.rpy_to_quaternions(left_target_rpy_, in_wxyz="wxyz", use_rad=False)

        right_target_pos_ = [0.15371069, -0.38635154602451355, -0.06846677]
        right_target_quat_ = [0.639313586157058, 0.6276110627003378, -0.35094137894395117, -0.2724383252702758]
        # # 发送消息
        insert_wedge_success = self.arm_controller.move_dual_arm_by_xyz_tr2(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_, steps=10, is_random=False, direction=[1.0, 0.0, 1.0])
        pass

    def get_arm_state(self):
        print("left_joint_positions =", list(self.arm_controller.left_joint_positions))
        print("left_end_effector_pose =", list(self.arm_controller.left_end_effector_pose))
        print("left_end_effector_quat =", list(self.arm_controller.left_end_effector_quat))
        print("right_joint_positions =", list(self.arm_controller.right_joint_positions))
        print("right_end_effector_pose =", list(self.arm_controller.right_end_effector_pose))
        print("right_end_effector_quat =", list(self.arm_controller.right_end_effector_quat))

    def test(self):
        # 置纸箱的逻辑
        left_target_pos_ = [0.15371069, 0.3863358589574437, -0.06846677]
        left_target_quat_ = [0.6394114697766845, -0.6273574282828561, -0.35111895992791475, 0.272563947497611]
        left_target_rpy_ = [-91.94376346, 0.14407546, 40.53020394]
        left_target_quat_ = self.trans.rpy_to_quaternions(left_target_rpy_, in_wxyz="wxyz", use_rad=False)
        # 发送消息
        self.arm_controller.move_single_arm_by_xyz(True, left_target_pos_, left_target_quat_)


if __name__ == "__main__":
    rospy.init_node("RobotControllerNode")
    robot_controller = RobotController()
    robot_controller.get_arm_state()
    # hand_init_success = robot_controller.hand_controller.init_hand_status()
    # move_left_status = robot_controller.arm_controller.rotate_joint([-0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0], [-0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    arm_init_success = robot_controller.arm_controller.init_arm_status()
    # robot_controller.insert_wedge()
    # robot_controller.grab_box()
    # robot_controller.place_box()
    robot_controller.test()
    print("Service test end")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""
@File    :   arms_ik.py
@Time    :   2025/07/10 10:07:49
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
"""
from scipy.spatial.transform import Rotation
from tracikpy import TracIKSolver
import numpy as np


class RobotIKSolver(TracIKSolver):
    def __init__(self, is_left=True):
        # 根据左右臂设置末端坐标系
        self.is_left = is_left
        urdf_file = "./description/tiangong_description/urdf/robot.urdf"
        tip_link = "wrist_roll_l_link" if self.is_left else "wrist_roll_r_link"
        base_link = "pelvis"
        # 调用父类初始化
        super().__init__(urdf_file, base_link=base_link, tip_link=tip_link)
        self.initial_angles = [0.0, 0.15, 0.0, -0.0, 0.0, 0.0, 0.0] if self.is_left else [0.0, -0.15, 0.0, -0.0, 0.0, 0.0, 0.0]

    def mat2quat_tf(self, matrix):
        """旋转矩阵转四元数（保持[w, x, y, z]格式）"""
        quat = Rotation.from_matrix(matrix).as_quat()
        return [quat[3], quat[0], quat[1], quat[2]]

    def quat_to_rot_matrix(self, quat):
        """四元数转旋转矩阵"""
        w, x, y, z = quat
        return np.array(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ]
        )

    def interpolate_position(self, start_pos, end_pos, num_points):
        """线性插值位置"""
        return np.linspace(start_pos, end_pos, num_points)

    def generate_trajectory_by_dist(self, start_pos_, end_pos_, dist_=0.02):
        """生成轨迹"""
        distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
        num_points = int(distance / dist_)
        return self.interpolate_position(start_pos_, end_pos_, num_points)

    def forward_kinematics(self, joint_angles) -> np.ndarray:
        """
        :param joint_angles: 7维关节角
        :return: 3维xyz坐标, 旋转矩阵, 四元数
        """
        if joint_angles is not None and len(joint_angles) == 7:
            joint_angles = np.concatenate(([0], joint_angles))
        ee_out = super().fk(joint_angles)
        xyz_ = ee_out[:3, 3]
        rot_ = ee_out[:3, :3]
        quat_ = self.mat2quat_tf(rot_)
        return xyz_, rot_, quat_

    def inverse_kinematics(self, target_position, target_quaternion=None, initial_angles=None, orientation_weight=1.0, use_rotation_matrix=False):
        ee_pose = np.eye(4)
        ee_pose[:3, 3] = np.array(target_position)
        if target_quaternion is None:
            cuurent_xyz_, current_rot_, target_quaternion = self.forward_kinematics(target_position)
            ee_pose[:3, :3] = self.quat_to_rot_matrix(target_quaternion)
        else:
            ee_pose[:3, :3] = self.quat_to_rot_matrix(target_quaternion)

        if initial_angles is not None and len(initial_angles) == 7:
            joints_pos_ = super().ik(ee_pose, np.concatenate(([0], initial_angles)))
            return joints_pos_[1:] if joints_pos_ is not None else initial_angles
        elif initial_angles is None:
            initial_angles = np.concatenate(([0], self.initial_angles))
            joints_pos_ = super().ik(ee_pose, initial_angles)
            return joints_pos_[1:] if joints_pos_ is not None else initial_angles
        else:
            raise ValueError("初始关节角数量错误")



if __name__ == "__main__":
    left_solver = RobotIKSolver(True)
    right_solver = RobotIKSolver(False)
    pos_xyz = np.array([0.32497879, 0.19681914, -0.06855335])
    quat_wxyz = np.array([0.6549775195272736, -0.5350869949072766, -0.36644698554585886, 0.38781820944787143])
    ik1_joints = left_solver.inverse_kinematics(pos_xyz, quat_wxyz)
    print("ik1", ik1_joints)
    pos_xyz = np.array([0.32497879, -0.19681914, -0.06855335])
    quat_wxyz = np.array([0.6549775195272736, 0.5350869949072766, -0.36644698554585886, -0.38781820944787143])
    ik2_joints = left_solver.inverse_kinematics(pos_xyz, quat_wxyz)
    print("ik2", ik2_joints)

    # 创建左臂和右臂的运动学对象
    left_arm = RobotIKSolver(is_left=True)
    right_arm = RobotIKSolver(is_left=False)

    # 初始关节角度
    left_joints = [-0.4157094955444336, -0.15037822723388672, 0.2577095031738281, -0.48166990280151367, 0.9909520149230957, 0.3322734832763672, -0.610745906829834]
    right_joints = [-0.4157094955444336, 0.15037822723388672, -0.2577095031738281, -0.48166990280151367, -.9909520149230957, 0.3322734832763672, 0.610745906829834]

    # 1. 正向运动学：计算末端位姿（位置和方向）
    left_pos, left_rot, left_quat = left_arm.forward_kinematics(left_joints)
    right_pos, right_rot, right_quat = right_arm.forward_kinematics(right_joints)

    print(f"左臂末端位置xyz: (相对于pelvis): {left_pos}")
    print(f"左臂末端四元数wxyz: {left_quat}")
    print(f"右臂末端位置xyz: (相对于pelvis): {right_pos}")
    print(f"右臂末端四元数wxyz: {right_quat}")

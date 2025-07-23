# import transforms3d as tf3d
from my_solver import MySolver
import numpy as np


if __name__ == "__main__":
    # 创建求解器
    arm_left_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True)
    arm_right_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_r_link", is_left=False)
    # 初始关节角度
    left_joints =  [0.0, 0.15, 0.0, -0.0, 0.0, 0.0, -0.0]
    right_joints =  [0.0, -0.15, 0.0, -0.0, 0.0, 0.0, -0.0]

    # 1. 正向运动学：计算末端位姿（位置和方向）
    left_pos, left_rot, left_quat = arm_left_kinematics.forward_kinematics(left_joints)
    right_pos, right_rot, right_quat = arm_left_kinematics.forward_kinematics(right_joints)

    print(f"left_pos = {list(left_pos)}")
    print(f"left_quat = {list(left_quat)}")
    print(f"right_pos = {list(right_pos)}")
    print(f"right_quat = {list(right_quat)}")

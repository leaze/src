# import transforms3d as tf3d
from my_solver import MySolver
import numpy as np


if __name__ == "__main__":
    # 创建求解器
    arm_left_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True)
    arm_right_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_r_link", is_left=False)
    # 初始关节角度
    left_joints =  [-0.504333764830546, -0.19203258438311485, 0.5687797544031549, -0.3358517591248969, 0.6184368370260153, 0.33441139366286493, -0.7184362322649265]
    right_joints =  [-0.5061886639850919, 0.19123308433375877, -0.569791921351373, -0.33175675879944155, -0.6171442084783899, 0.33267085294472976, 0.7196996180504094]

    # 1. 正向运动学：计算末端位姿（位置和方向）
    left_pos, left_rot, left_quat = arm_left_kinematics.forward_kinematics(left_joints)
    right_pos, right_rot, right_quat = arm_left_kinematics.forward_kinematics(right_joints)

    print(f"left_pos = {list(left_pos)}")
    print(f"left_quat = {list(left_quat)}")
    print(f"right_pos = {list(right_pos)}")
    print(f"right_quat = {list(right_quat)}")

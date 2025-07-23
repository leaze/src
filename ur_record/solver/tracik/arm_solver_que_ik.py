
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import make_interp_spline
from mpl_toolkits.mplot3d import Axes3D
from tracikpy import TracIKSolver
import matplotlib.pyplot as plt
import numpy as np
from arm_solver_que import ArmTracIKSolver



if __name__ == "__main__":
    arm_left_kinematics = ArmTracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link")
    arm_right_kinematics = ArmTracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_r_link")
    # 设置目标位姿
    left_pos = [5.0485000000000016e-05, 0.3474061668312111, -0.16362046604205055]
    left_quat = [0.9929713547651187, 0.11835492645396999, -0.0, 1.1102230246251565e-16]
    right_pos = [5.0485000000000016e-05, -0.3474061668312111, -0.16362046604205055]
    right_quat = [0.9929713547651187, -0.11835492645396999, 0.0, -1.1102230246251565e-16]
    init_left_joints = [0.0, 0.15, 0.0, -0.0, 0.0, 0.0, -0.0]
    init_right_joints = [0.0, -0.15, 0.0, -0.0, 0.0, 0.0, -0.0]
    while True:
        left_joints = arm_left_kinematics.inverse_kinematics(left_pos, left_quat, init_left_joints)
        right_joints = arm_right_kinematics.inverse_kinematics(right_pos, right_quat, init_right_joints)
        # print("left_joints = ", list(left_joints))
        # print("right_joints = ", list(right_joints))
        # 1. 正向运动学：计算末端位姿（位置和方向）
        valid_left_pos, valid_left_rot, valid_left_quat = arm_left_kinematics.forward_kinematics(left_joints)
        valid_right_pos, valid_right_rot, valid_right_quat = arm_right_kinematics.forward_kinematics(right_joints)
        left_xyz_diff = np.array(left_pos) - np.array(valid_left_pos)
        right_xyz_diff = np.array(right_pos) - np.array(valid_right_pos)
        print(f"left_diff = {list(left_xyz_diff)}")
        print(f"right_diff = {list(right_xyz_diff)}")
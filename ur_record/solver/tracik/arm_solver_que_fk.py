
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
    print(arm_left_kinematics.joint_names)
    print(arm_left_kinematics.lb)
    print(arm_left_kinematics.ub)
    # 初始关节角度
    left_joints =  [0.0, 0.35, 0.0, -0.0, 0.0, 0.0, -0.0]
    right_joints =  [0.0, -0.35, 0.0, -0.0, 0.0, 0.0, -0.0]
    
    # 1. 正向运动学：计算末端位姿（位置和方向）
    left_pos, left_rot, left_quat = arm_left_kinematics.forward_kinematics(left_joints)
    right_pos, right_rot, right_quat = arm_right_kinematics.forward_kinematics(right_joints)

    print(f"left_pos = {list(left_pos)}")
    print(f"left_quat = {list(left_quat)}")
    print(f"right_pos = {list(right_pos)}")
    print(f"right_quat = {list(right_quat)}")

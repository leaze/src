
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import make_interp_spline
from mpl_toolkits.mplot3d import Axes3D
from tracikpy import TracIKSolver
import matplotlib.pyplot as plt
import numpy as np
from arm_solver_que import ArmTracIKSolver



if __name__ == "__main__":
    arm_left_kinematics = ArmTracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link")
    # 设置目标位姿
    start_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    start_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    init_left_joints = None
    # 设置目标位姿
    end_pos = [0.2546261711182717, 0.15675063469266912, 0.07188115117764517]
    end_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    init_right_joints = None
    # 轨迹规划
    start_pose = arm_left_kinematics.create_pose(start_pos, start_quat)
    end_pose = arm_left_kinematics.create_pose(end_pos, end_quat)
    points = arm_left_kinematics.plan(start_pose, end_pose, 10, True, [1.0, 1.0, 1.0])

    # 正向运动学验证
    for point in points:
        # print("point = ", point)
        ik_joints = arm_left_kinematics.inverse_kinematics(point, start_quat)
        # print("ik_joints = ", list(ik_joints))
        fk_xyz_, _, _ = arm_left_kinematics.forward_kinematics(ik_joints)
        # print("fk_xyz_ = ", list(fk_xyz_))
        diff_ = np.linalg.norm(fk_xyz_ - point)
        print("diff_ = ", diff_)
    # 轨迹可视化
    arm_left_kinematics.visualize_trajectory(np.array(points))


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
    start_pos = [0.40593656521727806, 0.19103989555676446, 0.09998230819120782]
    start_quat = [0.6022336272331124, -0.5738677670383182, -0.45149258095975353, 0.3227148796110952]
    init_left_joints = None
    # 设置目标位姿
    end_pos = [0.15371069, 0.19029899, -0.06846677]
    end_quat = [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193]
    init_right_joints = None
    # 轨迹规划
    start_pose = arm_left_kinematics.create_pose(start_pos, start_quat)
    end_pose = arm_left_kinematics.create_pose(end_pos, end_quat)
    points, quats = arm_left_kinematics.plan(start_pose, end_pose, 10, False, [0.0, 0.0, 2.0])

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
    arm_left_kinematics.visualize_trajectory(np.array(points), np.array(quats))

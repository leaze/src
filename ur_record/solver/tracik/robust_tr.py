#!/usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation as R
from tracikpy import TracIKSolver
from scipy.spatial.transform import Slerp
from robust_solver import RobotController, quat_to_rot_matrix, visualize_trajectory


if __name__ == "__main__":
    # 创建求解器
    left_controller = RobotController(True)
    right_controller = RobotController(False)
    
    # 设置目标位姿
    left_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    left_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    init_left_joints = [-0.33386565065555185, -0.5402026304085296, 0.7349348901005469, -1.35786399744428, 0.594563784830394, 0.9201712176198895, -0.29331380180611144]
    
    # 生成轨迹
    all_points = []
    start_pose = np.eye(4)
    start_pose[:3, 3] = np.array(left_pos)
    start_pose[:3, :3] = quat_to_rot_matrix(left_quat)
    end_pose = np.eye(4)
    end_pose[:3, 3] = np.array([0.022746261711182717, 0.19675063469266912, -0.00188115117764517])
    end_pose[:3, :3] = quat_to_rot_matrix(left_quat)
    tr_joints, tr_points = left_controller.plan_path(start_pose, end_pose, steps=10)
    for tr_joint in tr_joints:
        left_joints = left_controller.move_to_pose(tr_joint, 10, 0.01)
        # init_left_joints = left_joints
        valid_pos = left_controller.ik_solver.base_solver.fk(left_joints)
        xyz_ = valid_pos[:3, 3]
        rot_ = valid_pos[:3, :3]
        quat_ = left_controller.mat2quat_tf(rot_)
        print("valid_pos = ", xyz_)
        all_points.append(xyz_)
    visualize_trajectory(np.array(all_points))
from my_solver import MySolver
import transforms3d as tf3d
import numpy as np





if __name__ == "__main__":
    # 创建求解器
    arm_left_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True)
    # 设置目标位姿
    start_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    start_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    end_pos = [0.2546261711182717, 0.15675063469266912, 0.07188115117764517]
    end_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]

    # 生成轨迹
    all_points = []
    start_pose = np.eye(4)
    start_pose[:3, 3] = np.array(start_pos)
    start_pose[:3, :3] = tf3d.quaternions.quat2mat(start_quat)
    end_pose = np.eye(4)
    end_pose[:3, 3] = np.array(end_pos)
    end_pose[:3, :3] = tf3d.quaternions.quat2mat(end_quat)
    tr_points, tr_quats = arm_left_kinematics.plan(start_pose, end_pose, steps=10, is_random=True, direction=[1.0, 1.0, 1.0])
    for i in range(len(tr_points)):
        tr_point = tr_points[i]
        tr_quat = tr_quats[i]
        ik_joints = arm_left_kinematics.inverse_kinematics(tr_point, tr_quat)
        fk_xyz, _, fk_quat = arm_left_kinematics.forward_kinematics(ik_joints)
        xyz_diff = np.linalg.norm(fk_xyz - tr_point)
        quat_diff = np.linalg.norm(fk_quat - tr_quat)
        ik_joints = arm_left_kinematics.inverse_kinematics(fk_xyz, fk_quat)
        print("diff = ", xyz_diff)
        
    arm_left_kinematics.visualize_trajectory(np.array(tr_points))

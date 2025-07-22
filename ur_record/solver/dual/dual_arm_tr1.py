from dual_arm_solver import ArmKinematics
import numpy as np


def generate_trajectory(start_pos_, end_pos_, num_points_=10):
    """生成直线轨迹"""
    return np.linspace(start_pos_, end_pos_, num_points_)


def generate_trajectory_by_dist(start_pos_, end_pos_, dist_=0.02):
    """生成轨迹"""
    distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
    num_points = int(distance / dist_)
    print("num_points", num_points)
    return generate_trajectory(start_pos_, end_pos_, num_points)


if __name__ == "__main__":
    # 创建左臂和右臂的运动学对象
    left_arm = ArmKinematics(is_left=True)
    right_arm = ArmKinematics(is_left=False)

    # 3. 轨迹规划：从位置和方向计算关节角度
    left_start_pos = [5.04850000e-05, 2.64192055e-01, -1.77546768e-01]
    left_start_quat = [9.99048232e-01, 4.36191563e-02, 0.00000000e00, -1.47352415e-16]
    left_end_pos = [0.28298403, 0.24302717, 0.06437022]
    left_end_quat = [0.706715, 0.03085568, -0.70615245, -0.03083112]
    trajectory = generate_trajectory_by_dist(left_start_pos, left_end_pos)
    for tr_point in trajectory:
        # 逆向运动学：从位置和方向计算关节角度
        left_joint_angles = left_arm.inverse_kinematics(tr_point, target_quaternion=left_end_quat, use_rotation_matrix=False)
        print(left_joint_angles)
        # 正向运动学：从关节角度计算位置和方向
        left_pos, left_rot, left_quat = left_arm.forward_kinematics(left_joint_angles)
        print(left_quat)

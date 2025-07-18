from coordinated_solver import StrictCoordinatedRobotIKSolver, quat_to_rot_matrix
import numpy as np

if __name__ == "__main__":
    # 创建左臂求解器
    left_arm = StrictCoordinatedRobotIKSolver("./description/tiangong_description/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True, symmetry_weight=10.0, max_attempts=15)  # 强约束
    # 创建右臂求解器
    right_arm = StrictCoordinatedRobotIKSolver("./description/tiangong_description/urdf/robot.urdf", "pelvis", "wrist_roll_r_link", is_left=False, symmetry_weight=10.0, max_attempts=15)  # 强约束
    # 2. 逆向运动学：从位置和方向计算关节角度
    left_pos = [0.32497879, 0.19681914, -0.06855335]
    left_quat = [0.65497752, -0.53508699, -0.36644699, 0.38781821]
    right_pos = [0.32497879, -0.19681914, -0.06855335]
    right_quat = [0.65497752, 0.53508699, -0.36644699, -0.38781821]

    # 左臂：使用位置和方向（四元数方法）
    left_joints = [0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
    recovered_left_joints = left_arm.inverse_kinematics(left_pos, target_quaternion=left_quat, use_rotation_matrix=False)

    # 右臂：使用位置和方向（旋转矩阵方法）
    recovered_right_joints = right_arm.inverse_kinematics(right_pos, target_quaternion=right_quat, use_rotation_matrix=False)

    print("left_joints = ", list(recovered_left_joints))
    print("right_joints = ", list(recovered_right_joints))

    # # 3. 正向运动学：从关节角度计算位置和方向
    # left_pos, _, left_quat = left_arm.forward_kinematics(recovered_left_joints)
    # right_pos, _, right_quat = right_arm.forward_kinematics(recovered_right_joints)
    # print("left_pos", left_pos)
    # print("left_quat", left_quat)
    # print("right_pos", right_pos)
    # print("right_quat", right_quat)
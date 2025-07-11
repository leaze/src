from dual_arm_solver import ArmKinematics
import numpy as np

if __name__ == "__main__":
    # 创建左臂和右臂的运动学对象
    left_arm = ArmKinematics(is_left=True)
    right_arm = ArmKinematics(is_left=False)

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

    print("recovered_left_joints", recovered_left_joints)
    print("recovered_right_joints", recovered_right_joints)

    # 3. 正向运动学：从关节角度计算位置和方向
    left_pos, _, left_quat = left_arm.forward_kinematics(recovered_left_joints)
    right_pos, _, right_quat = right_arm.forward_kinematics(recovered_right_joints)
    print("left_pos", left_pos)
    print("left_quat", left_quat)
    print("right_pos", right_pos)
    print("right_quat", right_quat)
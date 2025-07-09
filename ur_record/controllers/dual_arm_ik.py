from dual_arm_solver import ArmKinematics
import numpy as np

if __name__ == "__main__":
    # 创建左臂和右臂的运动学对象
    left_arm = ArmKinematics(is_left=True)
    right_arm = ArmKinematics(is_left=False)

    # 2. 逆向运动学：从位置和方向计算关节角度
    left_pos = [0.32415387, 0.20575715, -0.01802673]
    left_quat = [0.6458372786014214, -0.5310726198734097, -0.3764564706150609, 0.3989192997606842]
    right_pos = [5.04850000e-05, -1.66055208e-01, -1.89748507e-01]
    right_quat = [9.99048232e-01, 4.36191563e-02, 0.00000000e00, -1.47352415e-16]

    # 左臂：使用位置和方向（四元数方法）
    left_joints = [0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
    recovered_left_joints = left_arm.inverse_kinematics(left_pos, target_quaternion=left_quat, use_rotation_matrix=False)

    # 右臂：使用位置和方向（旋转矩阵方法）
    recovered_right_joints = right_arm.inverse_kinematics(right_pos, target_quaternion=right_quat, use_rotation_matrix=True)

    print("recovered_left_joints", recovered_left_joints)
    print("recovered_right_joints", recovered_right_joints)

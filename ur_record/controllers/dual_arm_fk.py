from dual_arm_solver import ArmKinematics
import numpy as np

if __name__ == "__main__":
    # 创建左臂和右臂的运动学对象
    left_arm = ArmKinematics(is_left=True)
    right_arm = ArmKinematics(is_left=False)

    # 初始关节角度
    left_joints = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
    right_joints = [0.0, -0.0, 0.0, -0.0, 0.0, 0.0, 0.0]

    # 1. 正向运动学：计算末端位姿（位置和方向）
    left_pos, left_rot, left_quat = left_arm.forward_kinematics(left_joints)
    right_pos, right_rot, right_quat = right_arm.forward_kinematics(right_joints)

    print(f"左臂末端位置xyz: (相对于pelvis): {left_pos}")
    print(f"左臂末端四元数wxyz: {left_quat}")
    print(f"右臂末端位置xyz: (相对于pelvis): {right_pos}")
    print(f"右臂末端四元数wxyz: {right_quat}")

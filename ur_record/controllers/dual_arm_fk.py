from dual_arm_solver import ArmKinematics
import numpy as np

if __name__ == "__main__":
    # 创建左臂和右臂的运动学对象
    left_arm = ArmKinematics(is_left=True)
    right_arm = ArmKinematics(is_left=False)

    # 初始关节角度
    left_joints =  [-0.504333764830546, -0.19203258438311485, 0.5687797544031549, -0.3358517591248969, 0.6184368370260153, 0.33441139366286493, -0.7184362322649265]
    right_joints =  [-0.5061886639850919, 0.19123308433375877, -0.569791921351373, -0.33175675879944155, -0.6171442084783899, 0.33267085294472976, 0.7196996180504094]

    # 1. 正向运动学：计算末端位姿（位置和方向）
    left_pos, left_rot, left_quat = left_arm.forward_kinematics(left_joints)
    right_pos, right_rot, right_quat = right_arm.forward_kinematics(right_joints)

    print(f"left_pos = {list(left_pos)}")
    print(f"left_quat = {list(left_quat)}")
    print(f"right_pos = {list(right_pos)}")
    print(f"right_quat = {list(right_quat)}")

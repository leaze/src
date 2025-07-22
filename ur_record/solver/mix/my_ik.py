# import transforms3d as tf3d
from my_solver import MySolver
import numpy as np


if __name__ == "__main__":
    # 创建求解器
    arm_left_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True)
    arm_right_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_r_link", is_left=False)
    # 设置目标位姿
    left_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    left_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    init_left_joints = None
    # 设置目标位姿
    right_pos = [0.32759669516187234, -0.1967146327303412, -0.07190695670671113]
    right_quat = [0.6549775332099196, 0.5350869754628191, -0.36644696956112155, -0.38781822827166285]
    init_right_joints = None
    if True:
        left_joints = arm_left_kinematics.inverse_kinematics(left_pos, left_quat, init_left_joints)
        right_joints = arm_right_kinematics.inverse_kinematics(right_pos, right_quat, init_right_joints)
        print("left_joints = ", list(left_joints))
        print("right_joints = ", list(right_joints))

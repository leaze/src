#!/usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation as R
from tracikpy import TracIKSolver
from scipy.spatial.transform import Slerp
from robust_solver import RobotController
        
if __name__ == "__main__":
    # 创建求解器
    left_controller = RobotController(True)
    right_controller = RobotController(False)
    
    # 设置目标位姿
    left_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    left_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    init_left_joints = [-0.33386565065555185, -0.5402026304085296, 0.7349348901005469, -1.35786399744428, 0.594563784830394, 0.9201712176198895, -0.29331380180611144]
    # 设置目标位姿
    right_pos = [0.32759669516187234, -0.1967146327303412, -0.07190695670671113]
    right_quat = [0.6549775332099196, 0.5350869754628191, -0.36644696956112155, -0.38781822827166285]
    init_right_joints = [-0.3220635774799992, 0.5284204253037288, -0.7083235187481794, -1.3571484416405197, -0.6327007667341071, 0.9092477393532518, 0.2705114985181885]
    while True:
        left_joints = left_controller.inverse_kinematics(left_pos, left_quat, init_left_joints)
        right_joints = right_controller.inverse_kinematics(right_pos, right_quat, init_right_joints)
        print("left_joints = ", list(left_joints))
        print("right_joints = ", list(right_joints))
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   main.py
@Time    :   2025/06/26 19:20:00
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg, MotorStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from controllers.dual_arm_solver import ArmKinematics
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from typing import List
import numpy as np
import rospy


class ArmController:
    def __init__(self):
        # 左右臂运动学求解器
        self.arm_left_kinematics = ArmKinematics(True)
        self.arm_right_kinematics = ArmKinematics(False)
        # 控制器参数
        self.control_rate = rospy.get_param("~control_rate", 50)  # Hz
        self.joint_tolerance = rospy.get_param("~joint_tolerance", 0.01)  # rad

        # 关节状态变量
        self.left_joint_positions = [0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.right_joint_positions = [0.0, -0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.dual_joint_positions = self.left_joint_positions + self.right_joint_positions
        self.left_end_effector_pose, self.left_end_effector_rota, self.left_end_effector_quat = (
            self.arm_left_kinematics.forward_kinematics(self.left_joint_positions)
        )
        self.right_end_effector_pose, self.right_end_effector_rota, self.right_end_effector_quat = (
            self.arm_right_kinematics.forward_kinematics(self.right_joint_positions)
        )
        self.joint_names = {"left": [i for i in range(11, 18)], "right": [j for j in range(21, 28)]}

        # 轨迹规划参数
        self.left_start_pose = None
        self.left_target_pose = None
        self.right_start_pose = None
        self.right_target_pose = None
        self.use_coordinated_motion = False

        # # 设置订阅者
        rospy.Subscriber("/arm/status", MotorStatusMsg, self.arm_status_callback)
        # rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        # rospy.Subscriber('/right_arm/target_pose', PoseStamped, self.right_target_callback)
        # rospy.Subscriber('/dual_arm/target_poses', PoseArray, self.coordinated_target_callback)

        # 设置发布者
        self.left_traj_pub = rospy.Publisher("/left_arm/joint_trajectory", JointTrajectory, queue_size=1)
        self.right_traj_pub = rospy.Publisher("/right_arm/joint_trajectory", JointTrajectory, queue_size=1)
        self.arm_cmd_pos_pub = rospy.Publisher("/arm/cmd_pos", CmdSetMotorPosition, queue_size=10)

    def arm_status_callback(self, arm_status_msg: MotorStatusMsg):
        for arm_idx_ in range(14):
            self.dual_joint_positions[arm_idx_] = arm_status_msg.status[arm_idx_].pos
        self.left_joint_positions = self.dual_joint_positions[0:7]
        self.right_joint_positions = self.dual_joint_positions[7:14]

    def send_arm_cmd_pos(self, name_: int, pos_: float, spd_: float, cur_: float):
        # 创建命令消息
        cmd_msg_ = CmdSetMotorPosition()
        cmd_msg_.header = Header(stamp=rospy.Time.now(), frame_id="")
        # 左臂：11---17, 右臂：21---27
        cmd1 = SetMotorPosition(name=name_, pos=pos_, spd=spd_, cur=cur_)  # 名称 # 弧度  # RPM  # 安培
        cmd_msg_.cmds = [cmd1]
        # 发布消息
        self.arm_cmd_pos_pub.publish(cmd_msg_)

    def send_arms_cmd_pos(self, name_ls: List[int], pos_ls: List[float], spd_ls: List[float], cur_ls: List[float]):
        if len(name_ls) != len(pos_ls) or len(name_ls) != len(spd_ls) or len(name_ls) != len(cur_ls):
            rospy.logerr("The length of name_ls, pos_ls, spd_ls, cur_ls must be equal.")
            return
        # 创建命令消息
        cmd_msgs_ = CmdSetMotorPosition()
        cmd_msgs_.header = Header(stamp=rospy.Time.now(), frame_id="")
        # 左臂：11---17, 右臂：21---27
        for i in range(len(name_ls)):
            cmd = SetMotorPosition(name=name_ls[i], pos=pos_ls[i], spd=spd_ls[i], cur=cur_ls[i])  # 名称 # 弧度  # RPM  # 安培
            cmd_msgs_.cmds.append(cmd)
        self.arm_cmd_pos_pub.publish(cmd_msgs_)

    def set_end_effector_target(self, is_left: bool, target_pos: list, target_ori: list, initial_angles: list, use_matrix: bool):

        if is_left:
            joint_angles_ = self.arm_left_kinematics.inverse_kinematics(target_pos, target_ori, initial_angles, use_matrix)
        else:
            joint_angles_ = self.arm_right_kinematics.inverse_kinematics(target_pos, target_ori, initial_angles, use_matrix)


if __name__ == "__main__":
    rospy.init_node("ArmController")
    controller = ArmController()
    print(controller.left_end_effector_pose)
    print(controller.left_end_effector_rota)
    print(controller.left_end_effector_quat)
    rospy.sleep(0.1)
    rospy.spin()

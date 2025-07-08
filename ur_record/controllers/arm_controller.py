#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""
@File    :   arm_controller.py
@Time    :   2025/06/26 19:20:00
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
"""
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
import math


class ArmController:
    def __init__(self):
        # 左右臂运动学求解器
        self.arm_left_kinematics = ArmKinematics(True)
        self.arm_right_kinematics = ArmKinematics(False)
        self.arm_kinematics = [self.arm_right_kinematics, self.arm_left_kinematics]

        # 控制器参数
        self.joint_speed = rospy.get_param("~joint_speed", 60)  # rpm
        self.joint_current = rospy.get_param("~joint_current", 10.0)  # A
        self.joint_tolerance = rospy.get_param("~joint_tolerance", 0.01)  # rad
        self.tr_distance = rospy.get_param("~tr_distance", 0.02)  # m
        self.tr_point_time = rospy.get_param("~tr_point_time", min(0.2, 2 * math.pi / self.joint_speed))  # s
        # 关节状态变量
        self.left_joint_status = {}
        self.left_joint_positions = [0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.right_joint_positions = [0.0, -0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.dual_joint_positions = [self.right_joint_positions, self.left_joint_positions]
        self.left_end_effector_pose, self.right_end_effector_rota, self.left_end_effector_quat = self.arm_left_kinematics.forward_kinematics(
            self.left_joint_positions
        )
        self.right_end_effector_pose, self.right_end_effector_rota, self.right_end_effector_quat = self.arm_right_kinematics.forward_kinematics(
            self.right_joint_positions
        )
        self.joint_names = {True: [i for i in range(11, 18)], False: [j for j in range(21, 28)]}

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
        self.left_tra_pub = rospy.Publisher("/left_arm/joint_trajectory", JointTrajectory, queue_size=1)
        self.right_tra_pub = rospy.Publisher("/right_arm/joint_trajectory", JointTrajectory, queue_size=1)
        self.arm_cmd_pos_pub = rospy.Publisher("/arm/cmd_pos", CmdSetMotorPosition, queue_size=10)

        # 初始化关节状态
        # self.init_arm_status()
        rospy.sleep(0.1)

    def arm_status_callback(self, arm_status_msg: MotorStatusMsg):
        name_ls_, pos_ls_, speed_ls_, current_ls_, temperature_ls_, error_ls_, dual_joint_positions_ls_ = [], [], [], [], [], [], [0.0] * 14
        for arm_idx_ in range(14):
            name_ls_.append(arm_status_msg.status[arm_idx_].name)
            pos_ls_.append(arm_status_msg.status[arm_idx_].pos)
            speed_ls_.append(arm_status_msg.status[arm_idx_].speed)
            current_ls_.append(arm_status_msg.status[arm_idx_].current)
            temperature_ls_.append(arm_status_msg.status[arm_idx_].temperature)
            error_ls_.append(arm_status_msg.status[arm_idx_].error)
            dual_joint_positions_ls_[arm_idx_] = arm_status_msg.status[arm_idx_].pos
        self.left_joint_positions = dual_joint_positions_ls_[0:7]
        self.right_joint_positions = dual_joint_positions_ls_[7:14]
        self.left_end_effector_pose, self.left_end_effector_rota, self.left_end_effector_quat = self.arm_left_kinematics.forward_kinematics(
            self.left_joint_positions
        )
        self.right_end_effector_pose, self.right_end_effector_rota, self.right_end_effector_quat = self.arm_right_kinematics.forward_kinematics(
            self.right_joint_positions
        )
        self.dual_joint_positions = [self.right_joint_positions, self.left_joint_positions]
        self.left_joint_status["name"] = name_ls_
        self.left_joint_status["pos"] = pos_ls_
        self.left_joint_status["speed"] = speed_ls_
        self.left_joint_status["current"] = current_ls_
        self.left_joint_status["temp"] = temperature_ls_
        self.left_joint_status["error"] = error_ls_

    def send_arm_cmd_pos(self, name_: int, pos_: float, spd_: float, cur_: float):
        """发送单个关节的目标位置命令
        :param name_: 关节名称 (int)
        :param pos_: 目标位置 (float, 弧度)
        :param spd_: 目标速度 (float, RPM)
        :param cur_: 目标电流 (float, 安培)
        """
        # 创建命令消息
        cmd_msg_ = CmdSetMotorPosition()
        cmd_msg_.header = Header(stamp=rospy.Time.now(), frame_id="")
        # 左臂：11---17, 右臂：21---27
        cmd1 = SetMotorPosition(name=name_, pos=pos_, spd=spd_, cur=cur_)  # 名称  # 弧度  # RPM  # 安培
        cmd_msg_.cmds.append(cmd1)
        # 发布消息
        self.arm_cmd_pos_pub.publish(cmd_msg_)

    def send_arms_cmd_pos(self, name_ls: List[int], pos_ls: List[float], spd_ls: List[float], cur_ls: List[float]):
        """发送多个关节的目标位置命令
        :param name_ls: 关节名称 (list)
        :param pos_ls: 目标位置 (list, 弧度)
        :param spd_ls: 目标速度 (list, RPM)
        :param cur_ls: 目标电流 (list, 安培)
        """
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

    def rotate_single_joint(self, is_left: bool, joint_name: int, rotate_angle: float):
        """旋转关节
        :param is_left: 是否为左臂 (bool)
        :param joint_name: 关节名称 (int), 这里填0-6即可, 分别对于7个关节
        :param joint_angles: 关节角度 (list, 弧度)
        """
        self.send_arm_cmd_pos(self.joint_names[is_left][joint_name], self.dual_joint_positions[is_left][joint_name] + rotate_angle, self.joint_speed, self.joint_current)
        rospy.sleep(self.tr_point_time)
        rotate_error = abs(self.dual_joint_positions[is_left][joint_name] + rotate_angle - self.dual_joint_positions[is_left][joint_name])
        rotate_status = rotate_error < self.joint_tolerance
        if rotate_status:
            rospy.loginfo("Rotate Joint %s %s: %s" % (self.joint_names[is_left][joint_name], rotate_angle, rotate_status))
        else:
            rospy.logerr("Rotate Joint %s %s: %s" % (self.joint_names[is_left][joint_name], rotate_angle, rotate_status))
        return rotate_status

    def set_end_effector_target(self, is_left: bool, target_pos: np.ndarray, target_ori: np.ndarray, use_matrix=False):
        # 生成轨迹(2厘米生成一个点, 减少数量以加快计算)
        start_pos, start_rot, start_quat = self.arm_kinematics[is_left].forward_kinematics(self.dual_joint_positions[is_left])
        positions = self.arm_kinematics[is_left].generate_trajectory_by_dist(start_pos, target_pos, self.tr_distance)
        # orientations = self.arm_kinematics[is_left].generate_trajectory_by_dist(start_quat, target_ori, self.tr_distance)
        if len(positions) == 0:
            rospy.logwarn("No trajectory points generated, check target position and distance.")
            positions = [target_pos]  # 如果没有生成点，则直接使用目标位置
        joint_trajectory = []
        all_positions = []
        # 跟踪前一步关节角度
        prev_joints = self.dual_joint_positions[is_left]
        for i, pos in enumerate(positions):
            # print(f"Position: {np.array_str(pos, precision=4, suppress_small=True)}")
            # print(f"Quaternion: {np.array_str(quat, precision=4, suppress_small=True)}")
            # 使用上一步的解作为初始值
            joint_angles_ = self.arm_kinematics[is_left].inverse_kinematics(
                pos,
                target_quaternion=target_ori,
                initial_angles=prev_joints,
                orientation_weight=1.0,
                use_rotation_matrix=use_matrix,
            )
            joint_trajectory.append(joint_angles_)
            # 正向运动学验证
            calc_pos_, calc_rot, calc_quat_ = self.arm_kinematics[is_left].forward_kinematics(joint_angles_)
            all_positions.append(calc_pos_)
            # print(f"FK Position xyz: {np.array_str(calc_pos_, precision=4, suppress_small=True)}")
            # print(f"FK Quaternion wxyz: {np.array_str(calc_quat_, precision=4, suppress_small=True)}")
            # print("Joint Angles (rad):", np.array(joint_angles_).round(2))
            # print("Joint Angles (deg):", np.rad2deg(joint_angles_).round(2))
        return joint_trajectory, all_positions

    def move_single_arm(self, is_left, target_pos, target_quat):
        trajectory_, all_points_ = self.set_end_effector_target(is_left, target_pos, target_quat)
        # 创建整个轨迹消息
        for j, (tr_, pos_) in enumerate(zip(trajectory_, all_points_)):
            speeds = [self.joint_speed] * 7  # RPM值
            currents = [self.joint_current] * 7  # 电流值(安培)
            self.send_arms_cmd_pos(self.joint_names[is_left], tr_, speeds, currents)
            rospy.sleep(self.tr_point_time)

    def move_dual_arm(self, left_target_pos, left_target_quat, right_target_pos, right_target_quat):
        # 分别计算左右臂的轨迹
        traj_left_joints_, traj_left_positions_ = self.set_end_effector_target(True, left_target_pos, left_target_quat)
        traj_right_joints_, traj_right_positions_ = self.set_end_effector_target(False, right_target_pos, right_target_quat)
        n_left = len(traj_left_joints_)
        n_right = len(traj_right_joints_)
        n_points = max(n_left, n_right)

        # 轨迹插值同步
        if n_left != n_right:
            # 时间归一化处理
            orig_index_left = np.linspace(0, 1, n_left)
            orig_index_right = np.linspace(0, 1, n_right)
            new_index = np.linspace(0, 1, n_points)

            # 插值处理
            synced_left = []
            for joint_idx in range(7):
                joint_values = [point[joint_idx] for point in traj_left_joints_]
                synced_values = np.interp(new_index, orig_index_left, joint_values)
                synced_left.append(synced_values)
            traj_left_joints_ = np.array(synced_left).T

            synced_right = []
            for joint_idx in range(7):
                joint_values = [point[joint_idx] for point in traj_right_joints_]
                synced_values = np.interp(new_index, orig_index_right, joint_values)
                synced_right.append(synced_values)
            traj_right_joints_ = np.array(synced_right).T

        # 同步执行双臂移动
        for i in range(n_points):
            # 合并左右臂的所有关节命令
            all_joint_names = self.joint_names[True] + self.joint_names[False]
            all_positions = traj_right_joints_[i].tolist() + traj_left_joints_[i].tolist()

            # 设置速度(根据位置差自适应调整)
            left_delta = np.linalg.norm(np.array(self.left_joint_positions) - np.array(traj_left_joints_[i]))
            right_delta = np.linalg.norm(np.array(self.right_joint_positions) - np.array(traj_right_joints_[i]))
            avg_speed = self.joint_speed + (left_delta + right_delta) * 0.5

            speeds = [min(self.joint_speed, max(0.1, avg_speed))] * 14
            currents = [self.joint_current] * 14

            # 发送命令
            self.send_arms_cmd_pos(all_joint_names, all_positions, speeds, currents)

            # 更新当前关节位置(用于下一步计算)
            # self.left_joint_positions = traj_left[i].tolist()
            # self.right_joint_positions = traj_right[i].tolist()

            rospy.sleep(self.tr_point_time)

        # left_plan_error_ = np.linalg.norm(np.array(left_target_pos) - np.array(traj_left_positions_[-1]))  # 左臂规划误差
        left_true_error2_ = np.linalg.norm(np.array(left_target_pos) - np.array(self.left_end_effector_pose))  # 左臂实际末端位置误差
        # right_plan_error_ = np.linalg.norm(np.array(right_target_pos) - np.array(traj_right_positions_[-1]))  # 右臂规划误差
        right_true_error2_ = np.linalg.norm(np.array(right_target_pos) - np.array(self.right_end_effector_pose))  # 右臂实际末端位置误差
        left_arm_move_status, right_arm_move_status = left_true_error2_ < self.joint_tolerance, right_true_error2_ < self.joint_tolerance
        rospy.loginfo("Arm Move Success") if left_arm_move_status and right_arm_move_status else rospy.loginfo("Arm Move Failed")

        return left_true_error2_ < self.joint_tolerance, right_true_error2_ < self.joint_tolerance

    def move_single_forward(self, is_left: bool, distance: float) -> bool:
        """移动单只臂"""
        start_pos_, start_rot_, start_quat_ = self.arm_kinematics[is_left].forward_kinematics(self.left_joint_positions)
        left_target_pos_ = start_pos_ + distance * np.array([1, 0, 0])  # 向前移动
        move_forward_single_success = self.move_single_arm(is_left, left_target_pos_, start_quat_)
        rospy.loginfo("Move Single Arm Forward Success") if move_forward_single_success else rospy.loginfo("Move Single Arm Forward Failed")
        return move_forward_single_success
    
    def move_single_backward(self, is_left: bool, distance: float) -> bool:
        """移动单只臂"""
        start_pos_, start_rot_, start_quat_ = self.arm_kinematics[is_left].forward_kinematics(self.left_joint_positions)
        left_target_pos_ = start_pos_ - distance * np.array([1, 0, 0])  # 向后移动
        move_backward_single_success = self.move_single_arm(is_left, left_target_pos_, start_quat_)
        rospy.loginfo("Move Single Arm Backward Success") if move_backward_single_success else rospy.loginfo("Move Single Arm Backward Failed")
        return move_backward_single_success
    
    def move_single_up(self, is_left: bool, distance: float) -> bool:
        """移动单只臂"""
        start_pos_, start_rot_, start_quat_ = self.arm_kinematics[is_left].forward_kinematics(self.left_joint_positions)
        left_target_pos_ = start_pos_ + distance * np.array([0, 0, 1])  # 向上移动
        move_up_single_success = self.move_single_arm(is_left, left_target_pos_, start_quat_)
        rospy.loginfo("Move Single Arm Up Success") if move_up_single_success else rospy.loginfo("Move Single Arm Up Failed")
        return move_up_single_success

    def move_single_down(self,  is_left: bool, distance: float) -> bool:
        """移动单只臂"""
        start_pos_, start_rot_, start_quat_ = self.arm_kinematics[is_left].forward_kinematics(self.left_joint_positions)
        left_target_pos_ = start_pos_ - distance * np.array([0, 0, 1])  # 向下移动
        move_down_single_success = self.move_single_arm(is_left, left_target_pos_, start_quat_)
        rospy.loginfo("Move Single Arm Down Success") if move_down_single_success else rospy.loginfo("Move Single Arm Down Failed")
        return move_down_single_success
    
    def move_single_left(self, is_left: bool, distance: float) -> bool:
        """移动单只臂"""
        start_pos_, start_rot_, start_quat_ = self.arm_kinematics[is_left].forward_kinematics(self.left_joint_positions)
        left_target_pos_ = start_pos_ - distance * np.array([0, 1, 0])  # 向左移动
        move_left_single_success = self.move_single_arm(is_left, left_target_pos_, start_quat_)
        rospy.loginfo("Move Single Arm Left Success") if move_left_single_success else rospy.loginfo("Move Single Arm Left Failed")
        return move_left_single_success
    
    def move_single_right(self, is_left: bool, distance: float) -> bool:
        """移动单只臂"""
        start_pos_, start_rot_, start_quat_ = self.arm_kinematics[is_left].forward_kinematics(self.left_joint_positions)
        left_target_pos_ = start_pos_ + distance * np.array([0, 1, 0])  # 向右移动
        move_right_single_success = self.move_single_arm(is_left, left_target_pos_, start_quat_)
        rospy.loginfo("Move Single Arm Right Success") if move_right_single_success else rospy.loginfo("Move Single Arm Right Failed")
        return move_right_single_success

    def move_forward(self, distance: float):
        """前进指定距离"""
        left_start_pos_, left_start_rot_, left_start_quat_ = self.arm_kinematics[True].forward_kinematics(self.left_joint_positions)
        right_start_pos_, right_start_rot_, right_start_quat_ = self.arm_kinematics[False].forward_kinematics(self.right_joint_positions)
        # 计算新的目标位置
        left_target_pos_ = left_start_pos_ + distance * np.array([1, 0, 0])  # 向前移动
        right_target_pos_ = right_start_pos_ + distance * np.array([1, 0, 0])  # 向前移动
        # 使用相同的姿态
        left_target_quat_ = left_start_quat_
        right_target_quat_ = right_start_quat_
        # 执行移动
        move_forward_left_success, move_forward_right_success = self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # 打印调试信息
        rospy.loginfo("Moved forward successfully") if move_forward_left_success and move_forward_right_success else rospy.logerr("Failed to move forward")
        return move_forward_left_success and move_forward_right_success


    def move_backward(self, distance: float) -> bool:
        """后退指定距离"""
        left_start_pos_, left_start_rot_, left_start_quat_ = self.arm_kinematics[True].forward_kinematics(self.left_joint_positions)
        right_start_pos_, right_start_rot_, right_start_quat_ = self.arm_kinematics[False].forward_kinematics(self.right_joint_positions)
        # 计算新的目标位置
        left_target_pos_ = left_start_pos_ - distance * np.array([1, 0, 0])  # 向后移动
        right_target_pos_ = right_start_pos_ - distance * np.array([1, 0, 0])  # 向后移动
        # 使用相同的姿态
        left_target_quat_ = left_start_quat_
        right_target_quat_ = right_start_quat_
        # 执行移动
        move_backward_left_success, move_backward_right_success = self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # 打印调试信息
        rospy.loginfo("Moved backward successfully") if move_backward_left_success and move_backward_right_success else rospy.logerr("Moved backward Failed")
        return move_backward_left_success and move_backward_right_success

    def move_up(self, distance: float) -> bool:
        """向上移动指定距离"""
        left_start_pos_, left_start_rot_, left_start_quat_ = self.arm_kinematics[True].forward_kinematics(self.left_joint_positions)
        right_start_pos_, right_start_rot_, right_start_quat_ = self.arm_kinematics[False].forward_kinematics(self.right_joint_positions)
        # 计算新的目标位置
        left_target_pos_ = left_start_pos_ + distance * np.array([0, 0, 1])  # 向上移动
        right_target_pos_ = right_start_pos_ + distance * np.array([0, 0, 1])  # 向上移动
        # 使用相同的姿态
        left_target_quat_ = left_start_quat_
        right_target_quat_ = right_start_quat_
        # 执行移动
        move_up_left_success, move_up_right_success = self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        rospy.loginfo("Moved up successfully") if move_up_left_success and move_up_right_success else rospy.logerr("Moved up Failed")
        # 打印调试信息
        return move_up_left_success and move_up_right_success

    def move_down(self, distance: float) -> bool:
        """向下移动指定距离"""
        left_start_pos_, left_start_rot_, left_start_quat_ = self.arm_kinematics[True].forward_kinematics(self.left_joint_positions)
        right_start_pos_, right_start_rot_, right_start_quat_ = self.arm_kinematics[False].forward_kinematics(self.right_joint_positions)
        # 计算新的目标位置
        left_target_pos_ = left_start_pos_ - distance * np.array([0, 0, 1])  # 向下移动
        right_target_pos_ = right_start_pos_ - distance * np.array([0, 0, 1])  # 向下移动
        # 使用相同的姿态
        left_target_quat_ = left_start_quat_
        right_target_quat_ = right_start_quat_
        # 执行移动
        move_down_left_success, move_down_right_success = self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # 打印调试信息
        rospy.loginfo("Moved down successfully") if move_down_left_success and move_down_right_success else rospy.logerr("Moved down Failed")
        return move_down_left_success and move_down_right_success

    def move_left(self, distance: float):
        """向左移动指定距离"""
        left_start_pos_, left_start_rot_, left_start_quat_ = self.arm_kinematics[True].forward_kinematics(self.left_joint_positions)
        right_start_pos_, right_start_rot_, right_start_quat_ = self.arm_kinematics[False].forward_kinematics(self.right_joint_positions)
        # 计算新的目标位置
        left_target_pos_ = left_start_pos_ - distance * np.array([0, 1, 0])
        right_target_pos_ = right_start_pos_ - distance * np.array([0, 1, 0])
        # 使用相同的姿态
        left_target_quat_ = left_start_quat_
        right_target_quat_ = right_start_quat_
        # 执行移动
        move_left_success, move_right_success = self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        rospy.loginfo("Moved left successfully") if move_left_success and move_right_success else rospy.logerr("Moved left Failed")
        return move_left_success and move_right_success

    def move_right(self, distance: float):
        """向右移动指定距离"""
        left_start_pos_, left_start_rot_, left_start_quat_ = self.arm_kinematics[True].forward_kinematics(self.left_joint_positions)
        right_start_pos_, right_start_rot_, right_start_quat_ = self.arm_kinematics[False].forward_kinematics(self.right_joint_positions)
        # 计算新的目标位置
        left_target_pos_ = left_start_pos_ + distance * np.array([0, 1, 0])
        right_target_pos_ = right_start_pos_ + distance * np.array([0, 1, 0])
        # 使用相同的姿态
        left_target_quat_ = left_start_quat_
        right_target_quat_ = right_start_quat_
        # 执行移动
        move_right_success, move_left_success = self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        rospy.loginfo("Moved right successfully") if move_right_success and move_left_success else rospy.logerr("Moved right Failed")
        return move_right_success and move_left_success

    def moves(self, distance: float, direction: str):
        """根据方向移动指定距离"""
        if direction == "forward":
            return self.move_forward(distance)
        elif direction == "backward":
            return self.move_backward(distance)
        elif direction == "up":
            return self.move_up(distance)
        elif direction == "down":
            return self.move_down(distance)
        elif direction == "left":
            return self.move_left(distance)
        elif direction == "right":
            return self.move_right(distance)
        else:
            rospy.logerr(f"Invalid direction: {direction}. Use 'forward', 'backward', 'up', or 'down'.")
            return False

    def init_arm_status(self):
        rospy.loginfo("Arm Controller Initializing...")
        all_joint_names = self.joint_names[True] + self.joint_names[False]
        all_positions = [0.0] * 14  # 初始化所有关节位置为0
        # all_positions[3] = -1.57
        # all_positions[10] = -1.57
        speeds = [self.joint_speed] * 14
        currents = [self.joint_current] * 14
        self.send_arms_cmd_pos(all_joint_names, all_positions, speeds, currents)
        rospy.sleep(2.0)
        dual_joint_error_ = np.linalg.norm(np.array(self.left_joint_positions + self.right_joint_positions) - np.array(all_positions))
        rospy.loginfo(f"Arm Initialization Status: {dual_joint_error_ < self.joint_tolerance}, dual joint error: {dual_joint_error_:.4f}")
        return dual_joint_error_ < self.joint_tolerance

    def control_loop(self):
        left_target_pos_ = [0.28298403, 0.24302717, 0.06437022]
        left_target_quat_ = [0.706715, 0.03085568, -0.70615245, -0.03083112]
        right_target_pos_ = [0.28298403, -0.18722009, 0.05216848]
        right_target_quat_ = [0.706715, 0.03085568, -0.70615245, -0.03083112]
        success_left, success_right = self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # self.move_single_arm(True, left_target_pos_, left_target_quat_)
        # self.move_single_arm(False, right_target_pos_, right_target_quat_)
        return success_left and success_right

    def run(self):
        """启动控制器"""
        try:
            rospy.loginfo("Starting dual arm control loop")
            self.control_loop()
        except rospy.ROSInterruptException:
            rospy.logerr("Dual arm controller interrupted")


if __name__ == "__main__":
    rospy.init_node("ArmController")
    controller = ArmController()
    controller.run()
    rospy.spin()

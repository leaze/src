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


class ArmController:
    def __init__(self):
        rospy.init_node("ArmController")
        # 左右臂运动学求解器
        self.arm_left_kinematics = ArmKinematics(True)
        self.arm_right_kinematics = ArmKinematics(False)
        self.arm_kinematics = [self.arm_right_kinematics, self.arm_left_kinematics]

        # 控制器参数
        self.control_rate = rospy.get_param("~control_rate", 50)  # Hz
        self.joint_tolerance = rospy.get_param("~joint_tolerance", 0.01)  # rad
        self.max_joint_speed = rospy.get_param("~max_joint_speed", 0.2)  # rad/s
        self.max_joint_acc = rospy.get_param("~max_joint_acc", 0.1)  # rad/s²
        self.max_joint_current = rospy.get_param("~max_joint_current", 1.0)  # A
        # self.max_joint_speed = 0.2  # rad/s
        # self.max_joint_accel = 0.1  # rad/s²
        # self.max_joint_current = 1.0  # A

        # 关节状态变量
        self.left_joint_status = {}
        self.left_joint_positions = [0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.right_joint_positions = [0.0, -0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.dual_joint_positions = self.left_joint_positions + self.right_joint_positions
        self.left_end_effector_pose, self.left_end_effector_rota, self.left_end_effector_quat = (
            self.arm_left_kinematics.forward_kinematics(self.left_joint_positions)
        )
        self.right_end_effector_pose, self.right_end_effector_rota, self.right_end_effector_quat = (
            self.arm_right_kinematics.forward_kinematics(self.right_joint_positions)
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

    def arm_status_callback(self, arm_status_msg: MotorStatusMsg):
        name_ls_, pos_ls_, speed_ls_, current_ls_, temperature_ls_, error_ls_ = [], [], [], [], [], []
        for arm_idx_ in range(14):
            name_ls_.append(arm_status_msg.status[arm_idx_].name)
            pos_ls_.append(arm_status_msg.status[arm_idx_].pos)
            speed_ls_.append(arm_status_msg.status[arm_idx_].speed)
            current_ls_.append(arm_status_msg.status[arm_idx_].current)
            temperature_ls_.append(arm_status_msg.status[arm_idx_].temperature)
            error_ls_.append(arm_status_msg.status[arm_idx_].error)
            self.dual_joint_positions[arm_idx_] = arm_status_msg.status[arm_idx_].pos
        self.left_joint_positions = self.dual_joint_positions[0:7]
        self.right_joint_positions = self.dual_joint_positions[7:14]
        self.left_joint_status["name"] = name_ls_
        self.left_joint_status["pos"] = pos_ls_
        self.left_joint_status["speed"] = speed_ls_
        self.left_joint_status["current"] = current_ls_
        self.left_joint_status["temp"] = temperature_ls_
        self.left_joint_status["error"] = error_ls_

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

    def set_end_effector_target(self, is_left: bool, target_pos: list, target_ori: list, use_matrix=False):
        # 生成轨迹(2厘米生成一个点, 减少数量以加快计算)
        start_pos, start_rot, start_quat = self.arm_kinematics[is_left].forward_kinematics(self.left_joint_positions)
        positions = self.arm_kinematics[is_left].generate_trajectory_by_dist(start_pos, target_pos, 0.02)
        # orientations = self.arm_kinematics[is_left].generate_trajectory_by_dist(start_quat, target_ori, 0.02)
        joint_trajectory = []
        all_positions = []
        # 跟踪前一步关节角度
        prev_joints = [self.left_joint_positions]
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
            # print(f"FK Position xyz: {np.array_str(calc_pos, precision=4, suppress_small=True)}")
            # print(f"FK Quaternion wxyz: {np.array_str(calc_quat_, precision=4, suppress_small=True)}")
            # print("Joint Angles (rad):", np.array(joint_angles_).round(2))
            # print("Joint Angles (deg):", np.rad2deg(joint_angles_).round(2))
        return joint_trajectory, all_positions

    def move_single_arm(self, is_left, target_pos, target_quat):
        trajectory_, all_points_ = self.set_end_effector_target(is_left, target_pos, target_quat)
        # 创建整个轨迹消息
        time_per_point = 0.2
        for j, (tr_, pos_) in enumerate(zip(trajectory_, all_points_)):
            speeds = [0.1] * 7  # RPM值
            currents = [0.1] * 7  # 电流值(安培)
            self.send_arms_cmd_pos(self.joint_names[is_left], tr_, speeds, currents)
            rospy.sleep(time_per_point)

    def move_dual_arm(self, left_target_pos, left_target_quat, right_target_pos, right_target_quat):
        # 分别计算左右臂的轨迹
        traj_left, _ = self.set_end_effector_target(True, left_target_pos, left_target_quat)
        traj_right, _ = self.set_end_effector_target(False, right_target_pos, right_target_quat)
        
        n_left = len(traj_left)
        n_right = len(traj_right)
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
                joint_values = [point[joint_idx] for point in traj_left]
                synced_values = np.interp(new_index, orig_index_left, joint_values)
                synced_left.append(synced_values)
            traj_left = np.array(synced_left).T
            
            synced_right = []
            for joint_idx in range(7):
                joint_values = [point[joint_idx] for point in traj_right]
                synced_values = np.interp(new_index, orig_index_right, joint_values)
                synced_right.append(synced_values)
            traj_right = np.array(synced_right).T
        
        # 同步执行双臂移动
        time_per_point = 0.2
        for i in range(n_points):
            # 合并左右臂的所有关节命令
            all_joint_names = self.joint_names[False] + self.joint_names[True]
            all_positions = traj_right[i].tolist() + traj_left[i].tolist()
            
            # 设置速度(根据位置差自适应调整)
            left_delta = np.linalg.norm(np.array(self.left_joint_positions) - np.array(traj_left[i]))
            right_delta = np.linalg.norm(np.array(self.right_joint_positions) - np.array(traj_right[i]))
            avg_speed = 0.1 + (left_delta + right_delta) * 0.5
            
            speeds = [min(self.max_joint_speed, max(0.1, avg_speed))] * 14
            currents = [0.1] * 14
            
            # 发送命令
            self.send_arms_cmd_pos(all_joint_names, all_positions, speeds, currents)
            
            # 更新当前关节位置(用于下一步计算)
            self.left_joint_positions = traj_left[i].tolist()
            self.right_joint_positions = traj_right[i].tolist()
            rospy.sleep(time_per_point)

    def control_loop(self):
        left_target_pos_ = [0.28298403, 0.24302717, 0.06437022]
        left_target_quat_ = [0.706715, 0.03085568, -0.70615245, -0.03083112]
        right_target_pos_ = [0.28298403, -0.18722009, 0.05216848]
        right_target_quat_ = [0.706715, 0.03085568, -0.70615245, -0.03083112]
        self.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        # self.move_single_arm(True, left_target_pos_, left_target_quat_)
        # self.move_single_arm(False, right_target_pos_, right_target_quat_)

    def run(self):
        """启动控制器"""
        try:
            rospy.loginfo("Starting dual arm control loop")
            self.control_loop()
        except rospy.ROSInterruptException:
            rospy.logerr("Dual arm controller interrupted")


if __name__ == "__main__":
    controller = ArmController()
    controller.run()
    rospy.sleep(0.1)
    rospy.spin()

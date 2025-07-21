#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   arm_6dof.py
@Time    :   2025/06/28 19:50:05
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from geometry_msgs.msg import WrenchStamped
import collections
import numpy as np
from std_msgs.msg import Int32
import rospy
import math
import time

class ForceControlState:
    POSITION_CONTROL = 0
    FORCE_CONTROL = 1
    HYBRID_CONTROL = 2
    COMPLIANT_MOVEMENT = 3

class Arm6Dof():
    def __init__(self):
        # 新增力控参数
        self.force_control_mode = ForceControlState.POSITION_CONTROL
        self.admittance_gain = rospy.get_param("~admittance_gain", 0.002)  # 导纳增益 (m/N)
        self.damping_factor = rospy.get_param("~damping_factor", 0.7)  # 阻尼系数
        self.force_threshold = rospy.get_param("~force_threshold", 5.0)  # 接触力阈值 (N)
        self.velocity_limit = rospy.get_param("~velocity_limit", 0.05)  # 最大调整速度 (m/s)
        
        # 手臂控制器参数
        self.tr_distance = rospy.get_param("~tr_distance", 0.02)  # m
        self.left_end_effector_pose = None
        self.right_end_effector_pose = None
        
        # 力传感器数据
        self.left_wrench = {"force": [0.0] * 3, "torque": [0.0] * 3, "header": [0]}
        self.right_wrench = {"force": [0.0] * 3, "torque": [0.0] * 3, "header": [0]}
        self.left_force_history = collections.deque(maxlen=10)  # 历史力数据
        self.right_force_history = collections.deque(maxlen=10)

        # 力控状态
        self.force_control_state_pub = rospy.Publisher('~force_control_state', Int32, queue_size=1)

        # 力传感器
        self.arm_6dof_left_status = [0.] * 6
        self.arm_6dof_right_status = [0.] * 6
        self.arm_6dof_left_sub = rospy.Subscriber("/arm_6dof_left", WrenchStamped, self.arm_6dof_left_callback)
        self.arm_6dof_right_sub = rospy.Subscriber("/arm_6dof_right", WrenchStamped, self.arm_6dof_right_callback)
        self.arm_6dof_timer = rospy.Timer(rospy.Duration(1), self.print_status_callback)
        rospy.sleep(0.1)

    def arm_6dof_left_callback(self, msg: WrenchStamped):
        self.arm_6dof_left_status = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        # rospy.loginfo("arm_6dof_left_status: %s", str(self.arm_6dof_left_status))
        """处理左臂力传感器数据"""
        self.left_wrench = {
            'force': np.array([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z
            ]),
            'torque': np.array([
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z
            ]),
            'header': msg.header
        }
        self.left_force_history.append(self.left_wrench['force'].copy())

    def print_status_callback(self, event):
        """定时器回调函数，打印当前状态"""
        # rospy.loginfo("arm_6dof_left_status: %s", str(self.arm_6dof_left_status))
        # rospy.loginfo("arm_6dof_left_status: %s", str(self.arm_6dof_right_status))
        pass
    def arm_6dof_right_callback(self, msg: WrenchStamped):
        self.arm_6dof_right_status = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        # rospy.loginfo("arm_6dof_right_status: %s", str(self.arm_6dof_right_status))
        """处理右臂力传感器数据"""
        self.right_wrench = {
            'force': np.array([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z
            ]),
            'torque': np.array([
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z
            ]),
            'header': msg.header
        }
        self.right_force_history.append(self.right_wrench['force'].copy())

    def has_recent_force_data(self, is_left):
        """检查是否有可用的力传感器数据"""
        if is_left:
            return True
            return (self.left_wrench is not None and 
                    (rospy.Time.now() - self.left_wrench['header'].stamp) < rospy.Duration(0.2))
        else:
            return True
            return (self.right_wrench is not None and 
                    (rospy.Time.now() - self.right_wrench['header'].stamp) < rospy.Duration(0.2))
    
    def publish_force_control_state(self):
        """发布力控状态"""
        state_msg = Int32()
        state_msg.data = self.force_control_mode
        self.force_control_state_pub.publish(state_msg)

    def apply_admittance_control(self, is_left):
        """
        基于导纳模型计算位置补偿
        返回: 位置补偿向量 (3D)
        """
        if not self.has_recent_force_data(is_left):
            return np.zeros(3)
            
        # 获取当前末端位姿
        if is_left:
            wrench = self.left_wrench
            current_pose = self.left_end_effector_pose
        else:
            wrench = self.right_wrench
            current_pose = self.right_end_effector_pose
            
        # 计算合力方向和大小
        contact_force = wrench['force']
        force_magnitude = np.linalg.norm(contact_force)
        
        # 只有超过阈值时才应用补偿
        if force_magnitude < self.force_threshold:
            return np.zeros(3)
            
        # 导纳模型: Δx = gain * (F_ext - damping * velocity)
        desired_velocity = self.admittance_gain * contact_force
        
        # 限幅
        desired_velocity_norm = np.linalg.norm(desired_velocity)
        if desired_velocity_norm > self.velocity_limit:
            desired_velocity = desired_velocity * self.velocity_limit / desired_velocity_norm
            
        # 阻尼处理 - 使运动更平滑
        desired_velocity *= (1.0 - self.damping_factor)
            
        return desired_velocity

    def move_dual_arm_by_xyz(self, left_target_pos, left_target_quat, right_target_pos, right_target_quat):
        """修改为支持双助力位混合控制"""
        # 应用左臂力补偿
        if self.force_control_mode in [ForceControlState.FORCE_CONTROL, ForceControlState.HYBRID_CONTROL]:
            left_adjustment = self.apply_admittance_control(True)
            right_adjustment = self.apply_admittance_control(False)
            
            left_target_pos += left_adjustment
            right_target_pos += right_adjustment
            
            rospy.logdebug("Left adj: %s, Right adj: %s", left_adjustment, right_adjustment)


    def compliant_move(self, is_left, target_pos, target_quat, max_force=15.0):
        # 阻抗控制
        """
        执行柔顺移动 - 在遇到阻力时自动调整
        max_force: 允许的最大接触力 (N)
        """
        original_mode = self.force_control_mode
        self.force_control_mode = ForceControlState.COMPLIANT_MOVEMENT
        self.force_threshold = max_force / 2.0  # 设置接触阈值
        self.publish_force_control_state()
        
        start_time = time.time()
        timeout = 10.0  # 秒
        success = False
        
        rospy.loginfo("Starting compliant movement...")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            # 获取当前力状态
            if is_left:
                current_force = np.linalg.norm(self.left_wrench['force']) if self.has_recent_force_data(True) else 0
            else:
                current_force = np.linalg.norm(self.right_wrench['force']) if self.has_recent_force_data(False) else 0
                
            # 检查是否超过力限
            if current_force > max_force:
                rospy.logwarn("Force threshold exceeded (%.1fN > %.1fN). Stopping movement.", 
                             current_force, max_force)
                break
            
            # 执行带力补偿的运动
            if is_left:
                self.move_single_arm_by_xyz(True, target_pos, target_quat)
            else:
                self.move_single_arm_by_xyz(False, target_pos, target_quat)
                
            # 检查是否到达目标
            if is_left:
                dist = np.linalg.norm(target_pos - self.left_end_effector_pose)
            else:
                dist = np.linalg.norm(target_pos - self.right_end_effector_pose)
                
            if dist < self.tr_distance:
                success = True
                rospy.loginfo("Compliant movement completed successfully.")
                break
                
            rospy.sleep(0.1)
            
        # 恢复原模式
        self.force_control_mode = original_mode
        self.publish_force_control_state()
        return success

if __name__ == "__main__":
    rospy.init_node("Arm6Dof")
    arm_6_dof = Arm6Dof()
    speed = arm_6_dof.apply_admittance_control(True)
    print(speed)
    rospy.sleep(0.1)
    rospy.spin()

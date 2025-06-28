#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   ex1.py
@Time    :   2025/06/28 19:43:16
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
import rospy
import numpy as np
from controllers.dual_arm_solver import ArmKinematics
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse

class DualArmController:
    def __init__(self):
        rospy.init_node('dual_arm_controller', anonymous=True)
        
        # 实例化左右臂运动学求解器
        self.arm_left_kinematics = ArmKinematics(True)
        self.arm_right_kinematics = ArmKinematics(False)
        
        # 控制器参数
        self.control_rate = rospy.get_param('~control_rate', 50)  # Hz
        self.joint_tolerance = rospy.get_param('~joint_tolerance', 0.01)  # rad
        
        # 关节状态变量
        self.left_joint_positions = None
        self.right_joint_positions = None
        self.joint_names = {"left": [], "right": []}
        
        # 轨迹规划参数
        self.left_target_pose = None
        self.right_target_pose = None
        self.use_coordinated_motion = False
        
        # 设置订阅者
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/left_arm/target_pose', PoseStamped, self.left_target_callback)
        rospy.Subscriber('/right_arm/target_pose', PoseStamped, self.right_target_callback)
        rospy.Subscriber('/dual_arm/target_poses', PoseArray, self.coordinated_target_callback)
        
        # 设置发布者
        self.left_traj_pub = rospy.Publisher('/left_arm/joint_trajectory', JointTrajectory, queue_size=1)
        self.right_traj_pub = rospy.Publisher('/right_arm/joint_trajectory', JointTrajectory, queue_size=1)
        
        # 设置服务
        rospy.Service('/dual_arm/start_motion', Trigger, self.start_motion_handler)
        rospy.Service('/dual_arm/stop_motion', Trigger, self.stop_motion_handler)
        
        # 初始化
        self.initialize_arms()
        rospy.loginfo("Dual Arm Controller initialized")

    def initialize_arms(self):
        """等待关节状态并获取初始配置"""
        rospy.loginfo("Waiting for initial joint states...")
        rospy.wait_for_message('/joint_states', JointState)
        rospy.loginfo("Received initial joint states")
        
        # 假设关节状态消息包含所有关节名
        # 实际应用中需要根据实际URDF配置提取左右臂关节名
        self.joint_names = {
            "left": ['left_shoulder_pan', 'left_shoulder_lift', ...],
            "right": ['right_shoulder_pan', 'right_shoulder_lift', ...]
        }

    def joint_states_callback(self, msg):
        """处理关节状态消息"""
        # 实际实现中需要根据关节名提取左右臂状态
        if self.joint_names["left"] and self.joint_names["right"]:
            self.left_joint_positions = [msg.position[msg.name.index(j)] for j in self.joint_names["left"]]
            self.right_joint_positions = [msg.position[msg.name.index(j)] for j in self.joint_names["right"]]

    def left_target_callback(self, msg):
        """左臂目标位姿回调"""
        self.left_target_pose = msg.pose
        rospy.loginfo("Received left arm target pose")
        self.use_coordinated_motion = False

    def right_target_callback(self, msg):
        """右臂目标位姿回调"""
        self.right_target_pose = msg.pose
        rospy.loginfo("Received right arm target pose")
        self.use_coordinated_motion = False

    def coordinated_target_callback(self, msg):
        """双臂协调位姿回调"""
        if len(msg.poses) == 2:
            self.left_target_pose = msg.poses[0]
            self.right_target_pose = msg.poses[1]
            rospy.loginfo("Received coordinated dual arm target poses")
            self.use_coordinated_motion = True

    def solve_dual_arm_ik(self, left_pose, right_pose):
        """求解双臂逆运动学"""
        try:
            # 求解左臂
            q_left = self.arm_left_kinematics.solve_ik(
                left_pose, 
                self.left_joint_positions,
                self.joint_tolerance
            )
            
            # 求解右臂
            q_right = self.arm_right_kinematics.solve_ik(
                right_pose, 
                self.right_joint_positions,
                self.joint_tolerance
            )
            
            return q_left, q_right
            
        except Exception as e:
            rospy.logerr(f"Dual arm IK failed: {str(e)}")
            return None, None

    def generate_trajectory(self, target_joints, current_joints, arm_name):
        """生成关节轨迹"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names[arm_name]
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = rospy.Duration(2.0)  # 2秒内到达
        
        trajectory.points = [point]
        return trajectory

    def execute_dual_arm_trajectory(self, left_trajectory, right_trajectory):
        """执行双臂轨迹"""
        # 发送轨迹
        self.left_traj_pub.publish(left_trajectory)
        self.right_traj_pub.publish(right_trajectory)
        rospy.loginfo("Dual arm trajectories published")

    def start_motion_handler(self, req):
        """启动运动服务处理"""
        # 实际实现中会有安全检查和状态确认
        response = TriggerResponse()
        response.success = True
        response.message = "Motion started"
        rospy.loginfo("Dual arm motion started")
        return response

    def stop_motion_handler(self, req):
        """停止运动服务处理"""
        # 实际实现中会有平滑停止逻辑
        response = TriggerResponse()
        response.success = True
        response.message = "Motion stopped"
        rospy.loginfo("Dual arm motion stopped")
        return response

    def control_loop(self):
        """主控制循环"""
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown():
            # 仅当有目标位姿且有关节状态时执行
            if (self.left_target_pose and self.right_target_pose and 
                self.left_joint_positions and self.right_joint_positions):
                
                # 求解逆运动学
                left_joints, right_joints = self.solve_dual_arm_ik(
                    self.left_target_pose, 
                    self.right_target_pose
                )
                
                if left_joints is not None and right_joints is not None:
                    # 生成轨迹
                    left_traj = self.generate_trajectory(
                        left_joints, 
                        self.left_joint_positions, 
                        "left"
                    )
                    
                    right_traj = self.generate_trajectory(
                        right_joints, 
                        self.right_joint_positions, 
                        "right"
                    )
                    
                    # 协调运动特定处理
                    if self.use_coordinated_motion:
                        self.adjust_for_coordinated_motion(left_traj, right_traj)
                    
                    # 执行轨迹
                    self.execute_dual_arm_trajectory(left_traj, right_traj)
                    
                    # 清除目标位姿
                    self.left_target_pose = None
                    self.right_target_pose = None
            
            rate.sleep()

    def adjust_for_coordinated_motion(self, left_traj, right_traj):
        """调整轨迹以实现双臂协调运动"""
        # 这里可以实现双臂同步或协作逻辑，例如：
        # 1. 使双臂轨迹时间同步
        # 2. 增加中间点实现复杂协作动作
        # 3. 计算协调操作的空间关系约束
        pass

    def run(self):
        """启动控制器"""
        try:
            rospy.loginfo("Starting dual arm control loop")
            self.control_loop()
        except rospy.ROSInterruptException:
            rospy.logerr("Dual arm controller interrupted")

if __name__ == '__main__':
    controller = DualArmController()
    controller.run()
import numpy as np
from scipy.optimize import minimize
import math

class ArmKinematics:
    def __init__(self, is_left=True):
        self.is_left = is_left
        self.sign = 1 if is_left else -1
        
        # 机械臂参数（从URDF提取）
        # 注意：所有位置都是相对于上一个关节的位置
        self.joint_params = [
            # shoulder_pitch (关节0)
            {"pos": np.array([0, self.sign * 0.14539, 0.37721]), 
             "rot_axis": np.array([0, 1, 0]),  # y轴
             "rot_rpy": np.array([0.087266, 0, 0])},  # 初始旋转
            
            # shoulder_roll (关节1)
            {"pos": np.array([0, self.sign * 0.068, 0]), 
             "rot_axis": np.array([1, 0, 0]),  # x轴
             "rot_rpy": np.array([0, 0, 0])},
            
            # shoulder_yaw (关节2)
            {"pos": np.array([0, 0, -0.1025]), 
             "rot_axis": np.array([0, 0, 1]),  # z轴
             "rot_rpy": np.array([0, 0, 0])},
            
            # elbow_pitch (关节3)
            {"pos": np.array([0.02, 0, -0.1975]), 
             "rot_axis": np.array([0, 1, 0]),  # y轴
             "rot_rpy": np.array([0, 0, 0])},
            
            # elbow_yaw (关节4)
            {"pos": np.array([-0.02, 0, -0.0543]), 
             "rot_axis": np.array([0, 0, 1]),  # z轴
             "rot_rpy": np.array([0, 0, 0])},
            
            # wrist_pitch (关节5)
            {"pos": np.array([0, self.sign * 0.002, -0.1887]), 
             "rot_axis": np.array([0, 1, 0]),  # y轴
             "rot_rpy": np.array([0, 0, 0])},
            
            # wrist_roll (关节6)
            {"pos": np.array([5.0485e-05, 0, -0.02]), 
             "rot_axis": np.array([1, 0, 0]),  # x轴
             "rot_rpy": np.array([0, 0, 0])},
        ]
        
        # 末端位置相对于最后一个关节的偏移
        self.end_effector_offset = np.array([0, 0, 0])
        
        # 关节限制 (从URDF提取)
        self.joint_limits = [
            (-2.96, 2.96),    # shoulder_pitch
            (-3.4, 0.2618) if is_left else (-0.2618, 3.4),  # shoulder_roll
            (-2.96, 2.96),    # shoulder_yaw
            (-2.61, 0.261),   # elbow_pitch
            (-2.9671, 2.9671),# elbow_yaw
            (-1.3, 1.65),     # wrist_pitch
            (-1.04, 0.785)    # wrist_roll
        ]
    
    def rotation_matrix(self, axis, angle):
        """创建绕指定轴的旋转矩阵"""
        axis = axis / np.linalg.norm(axis)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        t = 1 - cos_a
        
        return np.array([
            [t*axis[0]*axis[0] + cos_a,       t*axis[0]*axis[1] - sin_a*axis[2], t*axis[0]*axis[2] + sin_a*axis[1]],
            [t*axis[0]*axis[1] + sin_a*axis[2], t*axis[1]*axis[1] + cos_a,       t*axis[1]*axis[2] - sin_a*axis[0]],
            [t*axis[0]*axis[2] - sin_a*axis[1], t*axis[1]*axis[2] + sin_a*axis[0], t*axis[2]*axis[2] + cos_a]
        ])
    
    def rpy_matrix(self, rpy):
        """从滚转-俯仰-偏航创建旋转矩阵"""
        roll, pitch, yaw = rpy
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        
        return Rz @ Ry @ Rx
    
    def forward_kinematics(self, joint_angles):
        """计算正向运动学 - 给定关节角度计算末端位置"""
        T = np.eye(4)  # 从waist_yaw_link开始的齐次变换矩阵
        
        # 应用每个关节的变换
        for i, angle in enumerate(joint_angles):
            # 获取关节参数
            params = self.joint_params[i]
            
            # 创建旋转和平移矩阵
            R_joint = self.rotation_matrix(params["rot_axis"], angle)
            R_offset = self.rpy_matrix(params["rot_rpy"])
            R = R_joint @ R_offset
            
            # 创建齐次变换矩阵
            T_joint = np.eye(4)
            T_joint[:3, :3] = R
            T_joint[:3, 3] = params["pos"]
            
            # 累加变换
            T = T @ T_joint
        
        # 添加末端执行器偏移
        T_end = np.eye(4)
        T_end[:3, 3] = self.end_effector_offset
        T = T @ T_end
        
        # 返回位置
        position = T[:3, 3]
        return position
    
    def inverse_kinematics(self, target_position, initial_angles=None):
        """计算逆向运动学 - 给定目标位置计算关节角度"""
        # 设置初始角度（如果未提供）
        if initial_angles is None:
            initial_angles = [0] * len(self.joint_limits)
        
        # 定义优化目标函数
        def objective(joint_angles):
            current_position = self.forward_kinematics(joint_angles)
            error = np.linalg.norm(current_position - target_position)
            
            # 添加关节限位惩罚
            penalty = 0
            for i, angle in enumerate(joint_angles):
                low, high = self.joint_limits[i]
                if angle < low:
                    penalty += (low - angle) * 10.0
                elif angle > high:
                    penalty += (angle - high) * 10.0
            
            return error + penalty
        
        # 设置约束（关节限位）
        bounds = self.joint_limits
        
        # 使用优化方法求解
        result = minimize(objective, initial_angles, method='SLSQP', bounds=bounds)
        
        if result.success:
            return result.x
        else:
            # 如果优化失败，尝试使用不同的初始值
            for _ in range(5):
                new_initial = [np.random.uniform(low, high) for (low, high) in bounds]
                result = minimize(objective, new_initial, method='SLSQP', bounds=bounds)
                if result.success:
                    return result.x
            
            # 如果所有尝试都失败，返回最近似解
            return result.x


# 示例使用
if __name__ == "__main__":
    # 创建左臂和右臂的运动学对象
    left_arm = ArmKinematics(is_left=True)
    right_arm = ArmKinematics(is_left=False)
    
    # 初始关节角度
    left_joints = [0.0, 0.0, 0.0, -math.pi / 2, 0.0, 0.0, 0.0]
    right_joints = [0.0, -0.0, 0.0, -math.pi / 2, 0.0, 0.0, 0.0]
    
    # 1. 正向运动学：计算末端位置
    left_end_pos = left_arm.forward_kinematics(left_joints)
    right_end_pos = right_arm.forward_kinematics(right_joints)
    
    print(f"左臂末端位置 (相对于pelvis): {left_end_pos}")
    print(f"右臂末端位置 (相对于pelvis): {right_end_pos}")
    
    # 2. 逆向运动学：从位置计算关节角度
    recovered_left_joints = left_arm.inverse_kinematics(left_end_pos, left_joints)
    recovered_right_joints = right_arm.inverse_kinematics(right_end_pos, right_joints)
    
    print("\n恢复的左臂关节角度:")
    print(f"shoulder_pitch_l: {recovered_left_joints[0]:.4f}")
    print(f"shoulder_roll_l: {recovered_left_joints[1]:.4f}")
    print(f"shoulder_yaw_l: {recovered_left_joints[2]:.4f}")
    print(f"elbow_pitch_l: {recovered_left_joints[3]:.4f}")
    print(f"elbow_yaw_l: {recovered_left_joints[4]:.4f}")
    print(f"wrist_pitch_l: {recovered_left_joints[5]:.4f}")
    print(f"wrist_roll_l: {recovered_left_joints[6]:.4f}")
    
    print("\n恢复的右臂关节角度:")
    print(f"shoulder_pitch_r: {recovered_right_joints[0]:.4f}")
    print(f"shoulder_roll_r: {recovered_right_joints[1]:.4f}")
    print(f"shoulder_yaw_r: {recovered_right_joints[2]:.4f}")
    print(f"elbow_pitch_r: {recovered_right_joints[3]:.4f}")
    print(f"elbow_yaw_r: {recovered_right_joints[4]:.4f}")
    print(f"wrist_pitch_r: {recovered_right_joints[5]:.4f}")
    print(f"wrist_roll_r: {recovered_right_joints[6]:.4f}")
    
    # 验证逆向运动学结果
    print("\n验证逆向运动学:")
    left_pos_verif = left_arm.forward_kinematics(recovered_left_joints)
    print(f"原始左臂位置: {left_end_pos}")
    print(f"恢复的左臂位置: {left_pos_verif}")
    print(f"误差: {np.linalg.norm(left_end_pos - left_pos_verif)}")
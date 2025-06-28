import numpy as np
from scipy.optimize import minimize
import math
import transforms3d as tf3d

class ArmKinematics:
    def __init__(self, is_left=True):
        self.is_left = is_left
        self.sign = 1 if is_left else -1
        
        # 机械臂参数（从URDF提取）
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
    
    def normalize_quaternion(self, quat):
        """四元数归一化函数"""
        norm = np.linalg.norm(quat)
        if norm < 1e-10:
            return np.array([1.0, 0, 0, 0])
        return quat / norm
    
    def quat_to_rot_matrix(self, quat):
        """四元数转旋转矩阵 (可选)"""
        w, x, y, z = quat
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
    
    def forward_kinematics(self, joint_angles):
        """计算正向运动学 - 给定关节角度计算末端位姿（位置和方向）
        
        返回:
            tuple: (位置, 旋转矩阵, 四元数)
        """
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
        
        # 提取位置和旋转矩阵
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        
        # 将旋转矩阵转换为四元数
        try:
            # 使用transforms3d如果可用
            quaternion = tf3d.quaternions.mat2quat(rotation_matrix)
        except AttributeError:
            # 备选方案：手动计算四元数
            tr = rotation_matrix[0,0] + rotation_matrix[1,1] + rotation_matrix[2,2]
            if tr > 0:
                s = np.sqrt(tr + 1.0) * 2
                qw = 0.25 * s
                qx = (rotation_matrix[2,1] - rotation_matrix[1,2]) / s
                qy = (rotation_matrix[0,2] - rotation_matrix[2,0]) / s
                qz = (rotation_matrix[1,0] - rotation_matrix[0,1]) / s
            elif (rotation_matrix[0,0] > rotation_matrix[1,1]) and (rotation_matrix[0,0] > rotation_matrix[2,2]):
                s = np.sqrt(1.0 + rotation_matrix[0,0] - rotation_matrix[1,1] - rotation_matrix[2,2]) * 2
                qw = (rotation_matrix[2,1] - rotation_matrix[1,2]) / s
                qx = 0.25 * s
                qy = (rotation_matrix[0,1] + rotation_matrix[1,0]) / s
                qz = (rotation_matrix[0,2] + rotation_matrix[2,0]) / s
            elif rotation_matrix[1,1] > rotation_matrix[2,2]:
                s = np.sqrt(1.0 + rotation_matrix[1,1] - rotation_matrix[0,0] - rotation_matrix[2,2]) * 2
                qw = (rotation_matrix[0,2] - rotation_matrix[2,0]) / s
                qx = (rotation_matrix[0,1] + rotation_matrix[1,0]) / s
                qy = 0.25 * s
                qz = (rotation_matrix[1,2] + rotation_matrix[2,1]) / s
            else:
                s = np.sqrt(1.0 + rotation_matrix[2,2] - rotation_matrix[0,0] - rotation_matrix[1,1]) * 2
                qw = (rotation_matrix[1,0] - rotation_matrix[0,1]) / s
                qx = (rotation_matrix[0,2] + rotation_matrix[2,0]) / s
                qy = (rotation_matrix[1,2] + rotation_matrix[2,1]) / s
                qz = 0.25 * s
            
            quaternion = np.array([qw, qx, qy, qz])
        
        return position, rotation_matrix, quaternion
    
    def inverse_kinematics(self, target_position, target_quaternion=None, 
                          initial_angles=None, orientation_weight=1.0, 
                          use_rotation_matrix=False):
        """计算逆向运动学 - 给定目标位姿计算关节角度
        
        Args:
            target_position: 目标位置 (3D向量)
            target_quaternion: 目标方向 (四元数 [w, x, y, z])
            initial_angles: 初始关节角度猜测
            orientation_weight: 方向误差的权重系数
            use_rotation_matrix: 是否使用旋转矩阵而不是四元数计算方向误差
            
        Returns:
            np.array: 关节角度
        """
        # 设置初始角度
        if initial_angles is None:
            initial_angles = [0] * len(self.joint_limits)
        
        # 确保目标四元数已归一化
        if target_quaternion is not None:
            target_quaternion = self.normalize_quaternion(target_quaternion)
        
        # 定义优化目标函数
        def objective(joint_angles):
            # 计算当前位姿
            current_pos, current_rot, current_quat = self.forward_kinematics(joint_angles)
            
            # 位置误差
            pos_error = np.linalg.norm(current_pos - target_position)
            
            # 方向误差（如果指定了目标方向）
            ori_error = 0.0
            if target_quaternion is not None:
                # 归一化当前四元数
                current_quat_norm = self.normalize_quaternion(current_quat)
                
                if use_rotation_matrix:
                    # 方法1：使用旋转矩阵
                    target_rot = self.quat_to_rot_matrix(target_quaternion)
                    # 计算旋转矩阵之间的误差
                    rot_diff = current_rot @ target_rot.T
                    angle_error = np.arccos((np.trace(rot_diff) - 1) / 2)
                    ori_error = np.abs(angle_error)
                else:
                    # 方法2：使用四元数
                    # 计算四元数之间的点积
                    dot = np.dot(target_quaternion, current_quat_norm)
                    # 确保点积在有效范围内
                    dot = np.clip(dot, -1.0, 1.0)
                    # 计算角度差
                    ori_error = 2 * np.arccos(np.abs(dot))
            
            # 总误差
            total_error = pos_error + orientation_weight * ori_error
            
            # 添加关节限位惩罚
            penalty = 0
            for i, angle in enumerate(joint_angles):
                low, high = self.joint_limits[i]
                if angle < low:
                    penalty += (low - angle) * 10.0
                elif angle > high:
                    penalty += (angle - high) * 10.0
            
            return total_error + penalty
        
        # 设置约束（关节限位）
        bounds = self.joint_limits
        
        # 使用优化方法求解
        result = minimize(objective, initial_angles, method='SLSQP', bounds=bounds)
        
        if result.success:
            return result.x
        else:
            # 如果优化失败，尝试使用不同的初始值
            best_result = result.x
            best_error = objective(result.x)
            
            for _ in range(5):
                new_initial = [np.random.uniform(low, high) for (low, high) in bounds]
                new_result = minimize(objective, new_initial, method='SLSQP', bounds=bounds)
                if new_result.success:
                    return new_result.x
                new_error = objective(new_result.x)
                if new_error < best_error:
                    best_error = new_error
                    best_result = new_result.x
            
            # 返回最佳近似解
            return best_result


# 示例使用
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
    
    print(f"左臂末端位置 (相对于pelvis): {left_pos}")
    print(f"左臂末端四元数: {left_quat}")
    print(f"右臂末端位置 (相对于pelvis): {right_pos}")
    print(f"右臂末端四元数: {right_quat}")
    
    # 2. 逆向运动学：从位置和方向计算关节角度
    # 左臂：使用位置和方向（四元数方法）
    recovered_left_joints = left_arm.inverse_kinematics(
        left_pos, 
        target_quaternion=left_quat,
        initial_angles=left_joints,
        use_rotation_matrix=False
    )
    
    # 右臂：使用位置和方向（旋转矩阵方法）
    recovered_right_joints = right_arm.inverse_kinematics(
        right_pos, 
        target_quaternion=right_quat,
        initial_angles=right_joints,
        use_rotation_matrix=True
    )
    
    # 验证逆向运动学结果
    print("\n验证逆向运动学:")
    # 左臂
    left_pos_verif, _, left_quat_verif = left_arm.forward_kinematics(recovered_left_joints)
    pos_error_left = np.linalg.norm(left_pos - left_pos_verif)
    
    # 计算方向误差（角度差）
    dot = np.clip(np.dot(left_quat, left_quat_verif), -1.0, 1.0)
    ori_error_left = 2 * np.arccos(np.abs(dot))
    
    print(f"左臂位置误差: {pos_error_left:.6f}")
    print(f"左臂方向误差: {ori_error_left:.6f} 弧度")
    
    # 右臂
    right_pos_verif, _, right_quat_verif = right_arm.forward_kinematics(recovered_right_joints)
    pos_error_right = np.linalg.norm(right_pos - right_pos_verif)
    
    # 计算方向误差（角度差）
    dot = np.clip(np.dot(right_quat, right_quat_verif), -1.0, 1.0)
    ori_error_right = 2 * np.arccos(np.abs(dot))
    
    print(f"右臂位置误差: {pos_error_right:.6f}")
    print(f"右臂方向误差: {ori_error_right:.6f} 弧度")
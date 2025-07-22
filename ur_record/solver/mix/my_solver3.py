from tracikpy import TracIKSolver
import transforms3d as tf3d
import numpy as np
from scipy.linalg import solve
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R

class CustomIKSolver:
    def __init__(self, urdf_file, base_link, tip_link):
        """
        自定义逆运动学求解器 (基于TracIKSolver的物理模型)
        :param urdf_file: URDF文件路径
        :param base_link: 基座链接名称
        :param tip_link: 末端链接名称
        """
        self.solver = TracIKSolver(
            urdf_file,
            base_link=base_link,
            tip_link=tip_link,
            solve_type="Distance"
        )
        
        # 获取关节限制（去掉固定关节）
        self.lb, self.ub = self.solver.joint_limits
        self.lb, self.ub = self.lb[:], self.ub[:]  # 去除固定关节
        self.n_joints = len(self.lb)  # 实际关节数
        
        # 从URDF中获取关节ID映射
        self.joint_id = self.solver.link_names.index(tip_link)
        
        # 日志标志
        self.verbose = False
    
    def normalize_quaternion(self, quat):
        """四元数归一化函数"""
        norm = np.linalg.norm(quat)
        if norm < 1e-10:
            return np.array([1.0, 0, 0, 0])
        return quat / norm
    
    def log_se3(self, transform):
        """
        计算SE(3)变换矩阵的对数映射 (李代数)
        :param transform: 4x4齐次变换矩阵
        :return: 6维李代数向量 (平移, 旋转)
        """
        # 计算位置误差
        pos_error = transform[:3, 3]
        
        # 计算旋转矩阵误差 (对数映射)
        rot_error = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
        
        return np.concatenate([pos_error, rot_error])
    
    def forward_kinematics(self, joint_angles):
        """
        正运动学计算 (包含虚拟固定关节)
        :param joint_angles: 实际关节角度 (n_joints维)
        :return: 末端位置(3D), 旋转矩阵(3x3), 完整变换矩阵(4x4)
        """
        # 添加虚拟固定关节
        full_joints = np.concatenate(([0.0], joint_angles))
        full_joints = joint_angles
        # 计算正运动学
        transform = self.solver.fk(full_joints)
        
        # 提取位置和旋转
        pos = transform[:3, 3]
        rot = transform[:3, :3]
        
        return pos, rot, transform
    
    # def compute_jacobian(self, joint_angles):
    #     """
    #     计算空间雅可比矩阵 (6x(n_joints+1)，然后去除固定关节列)
    #     :param joint_angles: 实际关节角度 (n_joints维)
    #     :return: 6xn_joints雅可比矩阵
    #     """
    #     # 添加虚拟固定关节
    #     full_joints = np.concatenate(([0.0], joint_angles))
        
    #     # 计算完整雅可比矩阵 (包含固定关节)
    #     J_full = self.solver.jacobian(full_joints)
        
    #     # 删除固定关节对应的列 (第一列)
    #     J = J_full[:, 1:]
        
    #     return J
    def compute_jacobian(self, joints, delta=1e-6, adaptive_delta=True):
        """数值计算雅可比矩阵"""
        if joints is None:
            return None
        if joints is not None and len(joints) == 7:
            joints = np.concatenate(([0], joints))
        if adaptive_delta:
            # 基于当前位置自动调整delta
            current_pose = self.solver.fk(joints)
            pos_magnitude = np.linalg.norm(current_pose[:3,3])
            rot_magnitude = R.from_matrix(current_pose[:3,:3]).magnitude()
            delta = max(1e-6, min(0.01, 0.001 * pos_magnitude, 0.001 * rot_magnitude))

        jac = np.zeros((6, self.solver.number_of_joints))
        current_pose = self.solver.fk(joints)

        if current_pose is None:
            return None

        for i in range(self.solver.number_of_joints):
            # 扰动第i个关节
            perturbed_joints = joints.copy()
            perturbed_joints[i] += delta

            # 计算扰动后的位姿
            perturbed_pose = self.solver.fk(perturbed_joints)
            if perturbed_pose is None:
                continue

            # 计算位置差异
            pos_diff = (perturbed_pose[:3, 3] - current_pose[:3, 3]) / delta

            # 计算姿态差异（角度轴表示）
            r_current = R.from_matrix(current_pose[:3, :3])
            r_perturbed = R.from_matrix(perturbed_pose[:3, :3])
            rot_diff = (r_perturbed * r_current.inv()).as_rotvec() / delta
            # 增加平滑处理
            if np.linalg.norm(rot_diff) < 1e-8:
                rot_diff = np.zeros(3)

            # 组合雅可比列
            jac[:, i] = np.hstack([pos_diff, rot_diff])

        return jac
    
    def inverse_kinematics(self, current_q, target_rot, target_pos, max_iter=100, eps=1e-4, dt=0.01, damp=1e-6):
        """
        Pinocchio风格的逆运动学求解
        :param current_q: 当前关节角度 (n_joints维)
        :param target_rot: 目标旋转矩阵 (3x3)
        :param target_pos: 目标位置 (3维)
        :param max_iter: 最大迭代次数
        :param eps: 收敛阈值
        :param dt: 积分步长
        :param damp: 阻尼因子
        :return: 求解后的关节角度
        """
        # 创建目标变换矩阵
        target_transform = np.eye(4)
        target_transform[:3, :3] = target_rot
        target_transform[:3, 3] = target_pos
        
        q = current_q.copy()
        success = False
        
        for i in range(max_iter):
            # 计算当前正运动学
            _, _, current_transform = self.forward_kinematics(q)
            
            # 计算当前末端到目标末端的相对变换
            T_rel = np.linalg.inv(current_transform) @ target_transform
            
            # 使用李代数计算误差向量 (6维)
            err = self.log_se3(T_rel)
            err_norm = np.linalg.norm(err)
            
            if self.verbose and i % 10 == 0:
                print(f"Iter {i}: error = {err_norm:.6f}")
            
            # 检查收敛性
            if err_norm < eps:
                success = True
                break
            
            # 计算当前雅可比矩阵
            J = self.compute_jacobian(q)
            
            # 计算关节速度 (阻尼最小二乘法)
            # J_damped = J^T(JJ^T + damp*I)^-1
            # v = J_damped * err
            JJT = J @ J.T + damp * np.eye(6)
            v = J.T @ solve(JJT, err, assume_a='sym')
            
            # 更新关节角度 (简单欧拉积分)
            q += v * dt
            
            # 应用关节角度限制
            q = np.clip(q, self.lb, self.ub)
        
        if self.verbose:
            if success:
                # 最终误差检查
                _, _, current_transform = self.forward_kinematics(q)
                T_rel = np.linalg.inv(current_transform) @ target_transform
                final_err = np.linalg.norm(self.log_se3(T_rel))
                print(f"Convergence achieved after {i} iterations! Final error: {final_err:.6f}")
            else:
                print(f"Warning: Did not converge after {max_iter} iterations. Final error: {err_norm:.6f}")
        
        return q
    
    def inverse_kinematics_quat(self, current_q, target_quat, target_pos, **kwargs):
        """
        使用四元数作为输入的逆运动学
        :param target_quat: 目标四元数 [w, x, y, z]
        :其他参数同inverse_kinematics
        """
        # 将四元数转换为旋转矩阵
        target_quat = self.normalize_quaternion(target_quat)
        target_rot = tf3d.quaternions.quat2mat(target_quat)
        return self.inverse_kinematics(current_q, target_rot, target_pos, **kwargs)

# 使用示例
if __name__ == "__main__":
    # 初始化求解器
    ik_solver = CustomIKSolver(
        urdf_file='./ur_record/urdf/robot.urdf',
        base_link='pelvis',
        tip_link='wrist_roll_l_link'
    )
    
    # 设置日志
    ik_solver.verbose = True
    
    # 当前关节状态 (7维)
    current_q = np.array([0.0, 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    
    # 目标位置和方向 (旋转矩阵)
    target_pos = np.array([0.3, 0.1, 0.5])
    target_rot = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    
    # 计算逆运动学
    result = ik_solver.inverse_kinematics(
        current_q=current_q,
        target_rot=target_rot,
        target_pos=target_pos,
        max_iter=200,
        eps=1e-5
    )
    
    if result is not None:
        print("\n逆解算成功!")
        print(f"关节角度结果 (弧度): {result}")
        print(f"关节角度结果 (度): {np.degrees(result)}")
        
        # 验证结果
        pos, rot, _ = ik_solver.forward_kinematics(result)
        print("\n验证正运动学:")
        print(f"计算位置: {pos}")
        print(f"目标位置: {target_pos}")
        print(f"位置误差: {np.linalg.norm(pos - target_pos):.6f} meters")
        
        # 计算旋转矩阵差异 (Frobenius范数)
        rot_error = np.linalg.norm(rot - target_rot, 'fro')
        print(f"旋转矩阵误差: {rot_error:.6f}")
        
        # 使用四元数版本
        target_quat = tf3d.quaternions.mat2quat(target_rot)
        print("\n使用四元数接口计算...")
        result2 = ik_solver.inverse_kinematics_quat(
            current_q=current_q,
            target_quat=target_quat,
            target_pos=target_pos,
            max_iter=200,
            eps=1e-5
        )
        
        if np.allclose(result, result2, atol=1e-4):
            print("四元数接口计算结果一致!")
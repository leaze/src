from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation as R
from tracikpy import TracIKSolver
import matplotlib.pyplot as plt
import transforms3d as tf3d
import numpy as np


class ArmTracIKSolver(TracIKSolver):
    def __init__(self, urdf_file, base_link, tip_link, is_left=True, timeout=0.005, epsilon=1e-5, solve_type="Speed"):
        super().__init__(urdf_file, base_link, tip_link, timeout, epsilon, solve_type)
        self.lb, self.ub = self.joint_limits
        self.lb = self.lb * 0.92
        self.ub = self.ub * 0.92
        self.joint_mid = (self.lb + self.ub) / 2.0

    def rpy_to_rot(self, rpy, use_rad=True):
        """将欧拉角(roll, pitch, yaw)转换为四元数。
        参数:
            roll: 绕X轴的旋转角度(以度为单位)
            pitch: 绕Y轴的旋转角度(以度为单位)
            yaw: 绕Z轴的旋转角度(以度为单位)
            in_wxyz: 返回四元数的顺序，默认为"wxyz", 否则"xyzw"
            use_rad: 输入的roll, pitch, yaw参数是否为弧度, 否则为角度
        返回:
            四元数, 格式为numpy数组。
        """
        # 生成旋转对象（以XYZ欧拉角顺序）
        roll, pitch, yaw = rpy
        rot = R.from_euler("xyz", [roll, pitch, yaw]) if use_rad else R.from_euler("xyz", [np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)])
        return rot.as_matrix()

    def quaternion_to_rpy(self, quaternion, order="xyzw", use_rad=True):
        """
        将四元数转换为RPY角(弧度或角度)

        :param quaternion: 四元数, 长度为4的数组或列表
        :param order: 四元数的顺序，'xyzw'或'wxyz'
        :param use_rad: 若为True, 返回弧度; 否则返回角度
        :return: 以弧度或角度表示的RPY角(滚转、俯仰、偏航)
        """
        if order == "xyzw":
            q = quaternion
        elif order == "wxyz":
            # 转换成xyzw顺序，因为scipy默认使用xyzw
            q = [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
        else:
            raise ValueError("只支持'xyzw'或'wxyz'两种格式")

        r = R.from_quat(q)
        roll, pitch, yaw = r.as_euler("xyz")
        if use_rad:
            return np.array([roll, pitch, yaw])
        return np.degrees([roll, pitch, yaw])

    def create_pose(self, xyz_: list, rpy: list):
        ee_pose = np.eye(4)
        ee_pose[:3, 3] = np.array(xyz_)
        ee_pose[:3, :3] = self.rpy_to_rot(rpy)
        return ee_pose

    def forward_kinematics(self, joint_angles):
        """
        :param joint_angles: 7维关节角
        :return: 3维xyz坐标, 旋转矩阵, 四元数
        """
        if joint_angles is not None and len(joint_angles) == 7:
            joint_angles = np.concatenate(([0], joint_angles))
        ee_out = self.fk(joint_angles)
        xyz_ = ee_out[:3, 3]
        rot_ = ee_out[:3, :3]
        quat_ = tf3d.quaternions.mat2quat(rot_)
        return xyz_, rot_, quat_

    def inverse_kinematics(self, target_pose, qinit=None):
        solution = self.ik(target_pose, qinit)
        if solution is not None:
            return solution[:]  # 返回有效的关节角
        return solution[:] if solution is not None else qinit[:]

    def _compute_jacobian(self, joints, delta=1e-6, adaptive_delta=True):
        """数值计算雅可比矩阵"""
        if joints is None:
            return None
        if adaptive_delta:
            # 基于当前位置自动调整delta
            current_pose = self.fk(joints)
            pos_magnitude = np.linalg.norm(current_pose[:3, 3])
            rot_magnitude = R.from_matrix(current_pose[:3, :3]).magnitude()
            delta = max(1e-6, min(0.01, 0.001 * pos_magnitude, 0.001 * rot_magnitude))

        jac = np.zeros((6, self.number_of_joints))
        current_pose = self.fk(joints)

        if current_pose is None:
            return None

        for i in range(self.number_of_joints):
            # 扰动第i个关节
            perturbed_joints = joints.copy()
            perturbed_joints[i] += delta

            # 计算扰动后的位姿
            perturbed_pose = self.fk(perturbed_joints)
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

    def damped_least_squares_ik(self, target_pose, initial_joints, lambda_val=0.1, max_iter=100, tol=1e-5, adaptive_lambda=True):
        """
        使用阻尼最小二乘法求解逆运动学，带自适应步长控制
        :param target_pose: 目标位姿(4x4齐次矩阵)
        :param initial_joints: 初始关节角
        :param lambda_val: 基础阻尼系数
        :param max_iter: 最大迭代次数
        :param tol: 收敛容差
        :param adaptive_lambda: 是否启用自适应阻尼系数
        :return: 关节角
        """
        if initial_joints is None:
            initial_joints = self.joint_mid.copy()

        joints = initial_joints.copy()
        prev_error = float("inf")
        adaptive_lambda = lambda_val  # 初始阻尼值

        # 记录最优解
        best_joints = joints.copy()
        best_error = float("inf")

        for i in range(max_iter):
            # 计算当前位姿
            current_pose = self.fk(joints)
            if current_pose is None:
                # 使用之前的最佳解
                return best_joints if best_error < float("inf") else initial_joints

            # 计算位姿误差
            pos_error = target_pose[:3, 3] - current_pose[:3, 3]
            r_current = R.from_matrix(current_pose[:3, :3])
            r_target = R.from_matrix(target_pose[:3, :3])
            rot_error = (r_target * r_current.inv()).as_rotvec()
            error_vec = np.hstack([pos_error, rot_error])
            error_norm = np.linalg.norm(error_vec)

            # 更新最优解
            if error_norm < best_error:
                best_joints = joints.copy()
                best_error = error_norm

            # 检查收敛
            if error_norm < tol:
                break

            # 计算雅可比矩阵
            jac = self._compute_jacobian(joints)
            if jac is None or np.linalg.matrix_rank(jac) < 6:
                # 雅可比奇异，增加阻尼
                adaptive_lambda *= 2
                continue

            # 自适应阻尼系数（基于雅可比条件数）
            if adaptive_lambda:
                _, s, _ = np.linalg.svd(jac)
                cond = np.max(s) / np.min(s)
                adaptive_lambda = lambda_val * max(1.0, min(cond / 100, 5.0))

            # 计算阻尼最小二乘解
            jac_t = jac.T
            jjt = jac @ jac_t
            damping = adaptive_lambda * adaptive_lambda * np.eye(6)
            try:
                # 尝试直接求解
                delta_theta = jac_t @ np.linalg.solve(jjt + damping, error_vec)
            except np.linalg.LinAlgError:
                # 矩阵奇异，使用伪逆
                delta_theta = jac_t @ np.linalg.pinv(jjt + damping) @ error_vec

            # 自适应步长控制（回溯直线搜索）
            step_size = 1.0
            best_step_size = step_size
            min_step_error = error_norm

            # 尝试最多5个步长
            for _ in range(5):
                new_joints = joints + step_size * delta_theta
                new_joints = np.clip(new_joints, self.lb, self.ub)

                # 计算新位置的误差
                new_pose = self.fk(new_joints)
                if new_pose is None:
                    step_size *= 0.5
                    continue

                new_pos_error = target_pose[:3, 3] - new_pose[:3, 3]
                new_rot_error = (r_target * R.from_matrix(new_pose[:3, :3]).inv()).as_rotvec()
                new_error = np.linalg.norm(np.hstack([new_pos_error, new_rot_error]))

                # 检查是否改善
                if new_error < min_step_error:
                    min_step_error = new_error
                    best_step_size = step_size

                # 如果已经显著改善或步长太小，则退出
                if new_error < error_norm * 0.9 or step_size < 0.1:
                    break

                step_size *= 0.7  # 减小步长

            # 使用最佳步长更新关节
            joints += best_step_size * delta_theta
            joints = np.clip(joints, self.lb, self.ub)

            # 检查误差变化率
            error_reduction = prev_error - min_step_error
            if error_reduction < tol * 0.1 and i > 10:
                break
            prev_error = min_step_error

        # 验证最终解
        final_pose = self.fk(joints)
        if final_pose is not None:
            final_pos_error = np.linalg.norm(target_pose[:3, 3] - final_pose[:3, 3])
            final_rot_error = (r_target * R.from_matrix(final_pose[:3, :3]).inv()).magnitude()
            if final_pos_error < 0.01 and final_rot_error < 0.1:
                return joints

        # 如果最终解不理想，返回最佳中间解
        return best_joints

    def slerp_orientations(self, q1, q2, steps):
        """四元数球面线性插值"""
        t = np.linspace(0, 1, steps)
        orientations = []

        for i in range(steps):
            dot = np.dot(q1, q2)
            dot = np.clip(dot, -1.0, 1.0)
            theta = np.arccos(dot) * t[i]

            q = q2 - q1 * dot
            norm_q = np.linalg.norm(q)
            if norm_q > 1e-8:  # 设置很小的阈值
                q = q / norm_q
            else:
                # 如果范数太小，可能直接用其中一个四元数
                q = np.zeros_like(q)

            result = q1 * np.cos(theta) + q * np.sin(theta)
            # 归一化结果，保证正确性
            norm_result = np.linalg.norm(result)
            if norm_result > 1e-8:
                result = result / norm_result

            orientations.append(result)

        return orientations

    def plan(self, start_pose, end_pose, steps=10, is_random=False, direction=[1.0, 1.0, 1.0]):
        """
        生成起点到终点的非直线轨迹(三维B样条)

        参数:
            start_pose: 起点位姿 (geometry_msgs/Pose)
            end_pose: 终点位姿 (geometry_msgs/Pose)
            steps: 路径点数量（包括起点终点）
            direction: 轨迹方向向量 (list of float)

        返回:
            path: 轨迹点位姿列表 (list of geometry_msgs/Pose)
        """
        # 提取起点终点坐标
        start_pos = start_pose[:3, 3]
        end_pos = end_pose[:3, 3]
        r_start = R.from_matrix(start_pose[:3, :3])
        r_end = R.from_matrix(end_pose[:3, :3])
        if is_random:
            # 生成曲形控制点 (在起点终点之间添加偏移控制点)
            mid1 = start_pos + (end_pos - start_pos) * 0.3 + np.random.uniform(-0.01, 0.01, 3) * np.array(direction)  # 添加偏移控制点
            mid2 = start_pos + (end_pos - start_pos) * 0.7 + np.random.uniform(-0.01, 0.01, 3) * np.array(direction)  # 添加偏移控制点
        else:
            # 指定特定偏移量（非随机）
            mid1 = start_pos + (end_pos - start_pos) * 0.3 + np.array([0.02, -0.02, 0.02] * np.array(direction))  # 添加偏移控制点
            mid2 = start_pos + (end_pos - start_pos) * 0.7 + np.array([0.02, -0.02, 0.02] * np.array(direction))  # 添加偏移控制点

        control_points = np.vstack([start_pos, mid1, mid2, end_pos])
        # 参数化控制点
        t = [0, 0.3, 0.7, 1.0]
        spline = make_interp_spline(t, control_points, k=3)

        # 生成平滑轨迹
        u = np.linspace(0, 1, steps)
        positions = spline(u)

        # 姿态插值（四元数球面线性插值）
        start_quat = r_start.as_quat()
        end_quat = r_end.as_quat()
        orientations = self.slerp_orientations(start_quat, end_quat, steps)
        # 构建路径点
        all_points = []
        all_quats = []
        all_rpys = []
        for i in range(steps):
            all_points.append(positions[i])
            # all_quats.append(orientations[i])
            all_rpys.append(self.quaternion_to_rpy(orientations[i], "xyzw", use_rad=True))
        return all_points, all_rpys  # 返回路径点xyz, 欧拉角rpy
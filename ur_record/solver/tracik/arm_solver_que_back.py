#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""
@File    :   arm_solver.py
@Time    :   2025/07/20 13:49:38
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
"""
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import make_interp_spline
from mpl_toolkits.mplot3d import Axes3D
from tracikpy import TracIKSolver
import matplotlib.pyplot as plt
import numpy as np


class ArmTracIKSolver(TracIKSolver):
    def __init__(self, urdf_file, base_link, tip_link, timeout=0.005, epsilon=1e-5, solve_type="Speed", max_attempts=10):
        super().__init__(urdf_file, base_link, tip_link, timeout, epsilon, solve_type)
        self.lb, self.ub = self.joint_limits
        self.joint_mid = (self.lb + self.ub) / 2.0
        self.current_joints = [0.0] * self.number_of_joints
        self.obstacle_detected = False
        self.solution_queue = []  # 新增：求解结果队列
        self.queue_size = 10  # 队列容量

    def robust_ik(self, ee_pose, current_joints=None, max_attempts=20, tol=1e-3, singularity_threshold=0.001):
        """
        鲁棒的逆运动学求解，确保解在物理限位内、接近当前位置且远离奇异值

        :param ee_pose: 目标末端位姿(4x4齐次矩阵)
        :param current_joints: 当前关节位置(用于选择最近解)
        :param max_attempts: 最大求解尝试次数
        :param tol: 关节空间相似度容差
        :param singularity_threshold: 奇异值检测阈值
        :return: 最优关节角或None
        """
        valid_solutions = []

        # 尝试使用当前关节位置作为种子
        if current_joints is not None:
            solution = self.ik(ee_pose, qinit=current_joints)
            if self._is_valid(solution, tol, valid_solutions):
                valid_solutions.append(solution)

        # 尝试使用中间位置作为种子
        if not valid_solutions:
            solution = self.ik(ee_pose, qinit=self.joint_mid)
            if self._is_valid(solution, tol, valid_solutions):
                valid_solutions.append(solution)

        # 随机种子尝试
        attempt_count = 0
        while len(valid_solutions) < 3 and attempt_count < max_attempts:
            qinit = np.random.default_rng().uniform(self.lb, self.ub)
            solution = self.ik(ee_pose, qinit=qinit)
            if self._is_valid(solution, tol, valid_solutions):
                valid_solutions.append(solution)
            attempt_count += 1

        # 如果没有有效解
        if not valid_solutions:
            return None

        # 过滤掉接近奇异的解
        non_singular_solutions = []
        for sol in valid_solutions:
            if not self.is_near_singularity(sol, singularity_threshold):
                non_singular_solutions.append(sol)

        # 如果没有非奇异解，则使用最不奇异的解
        if not non_singular_solutions:
            print("Warning: All solutions near singularity. Selecting least singular.")
            # 计算每个解的奇异值
            singular_values = [np.min(np.linalg.svd(self._compute_jacobian(sol))[1]) for sol in valid_solutions]
            # 选择最大奇异值（最不奇异）的解
            best_idx = np.argmax(singular_values)
            return valid_solutions[best_idx]

        return self._select_best_solution(non_singular_solutions, ee_pose, current_joints)

    def is_near_singularity(self, joints, relative_threshold=0.05):
        """
        检测是否接近奇异位形
        :param joints: 当前关节角
        :param relative_threshold: 奇异值检测阈值（最小奇异值）
        :return: 布尔值，表示是否接近奇异
        """
        if joints is None:
            return False

        jac = self._compute_jacobian(joints)
        if jac is None: 
            return True
        
        _, s, _ = np.linalg.svd(jac)
        
        # 改进1：条件数检测
        condition_number = np.max(s) / np.min(s)
        if condition_number > 1/(relative_threshold):
            return True
        
        # 改进2：最小奇异值动态阈值
        dynamic_threshold = max(relative_threshold, 0.01 * np.median(s))
            
        return np.min(s) < dynamic_threshold

    def queue_based_ik(self, ee_pose, current_joints=None, max_attempts=20, 
                      tol=1e-3, singularity_threshold=0.001):
        """
        基于队列的鲁棒逆运动学求解
        :param ee_pose: 目标末端位姿(4x4齐次矩阵)
        :param current_joints: 当前关节位置
        :param max_attempts: 最大求解尝试次数
        :param tol: 关节空间相似度容差
        :param singularity_threshold: 奇异值检测阈值
        :return: 最优关节角或None
        """
        # 清空无效解
        self._prune_queue()
        
        # 生成新解集
        new_solutions = self._generate_solutions(
            ee_pose, current_joints, max_attempts, tol, singularity_threshold)
        
        # 添加到队列
        self.solution_queue.extend(new_solutions)
        
        # 筛选最优解
        best_solution = self._select_queue_best(current_joints, ee_pose)
        return best_solution
    
    def _generate_solutions(self, ee_pose, current_joints, max_attempts, tol, singularity_threshold):
        """生成多种子求解结果"""
        solutions = []
        
        # 1. 当前关节种子
        if current_joints is not None:
            sol = self.ik(ee_pose, qinit=current_joints)
            if self._is_valid(sol, tol, solutions):
                solutions.append(sol)
        
        # 2. 关节中位种子
        sol = self.ik(ee_pose, qinit=self.joint_mid)
        if self._is_valid(sol, tol, solutions):
            solutions.append(sol)
        
        # 3. 随机种子（至少5个）
        attempt_count = 0
        while len(solutions) < 5 and attempt_count < max_attempts:
            qinit = np.random.default_rng().uniform(self.lb, self.ub)
            sol = self.ik(ee_pose, qinit=qinit)
            if self._is_valid(sol, tol, solutions):
                solutions.append(sol)
            attempt_count += 1
        
        # 4. 历史最优解种子（如果存在）
        if self.solution_queue:
            best_historical = min(self.solution_queue, 
                                 key=lambda x: x['score'] if 'score' in x else float('inf'))
            sol = self.ik(ee_pose, qinit=best_historical['joints'])
            if self._is_valid(sol, tol, solutions):
                solutions.append(sol)
        
        return solutions
    
    def _prune_queue(self):
        """清理队列：移除旧解和无效解"""
        # 保持队列长度
        if len(self.solution_queue) > self.queue_size:
            # 按分数排序，保留最优解
            self.solution_queue.sort(key=lambda x: x.get('score', float('-inf')), reverse=True)
            self.solution_queue = self.solution_queue[:self.queue_size]
        
        # 移除超出限位的解
        valid_queue = []
        for sol in self.solution_queue:
            if isinstance(sol, dict):
                joints = sol['joints']
            else:
                joints = sol
                
            if not self._is_out_of_bounds(joints):
                valid_queue.append(sol)
        self.solution_queue = valid_queue
    
    def _select_queue_best(self, current_joints, target_pose):
        """从队列中选择最优解"""
        if not self.solution_queue:
            return None
        
        # 为每个解评分
        for sol in self.solution_queue:
            # 确保 sol 是字典类型
            if isinstance(sol, dict):
                if 'score' not in sol:
                    sol['score'] = self._calculate_solution_score(
                        sol['joints'], current_joints, target_pose)
            else:
                # 如果 sol 不是字典，转换为字典格式
                sol_dict = {'joints': sol, 'score': self._calculate_solution_score(
                    sol, current_joints, target_pose)}
                self.solution_queue = [s for s in self.solution_queue if not np.array_equal(s, sol)]
                self.solution_queue.append(sol_dict)
        
        # 选择最高分的解
        best_solution = max(self.solution_queue, key=lambda x: x['score'])
        return best_solution['joints']
    
    def _calculate_solution_score(self, joints, current_joints, target_pose):
        """计算解的综合性评分"""
        scores = []
        
        # 1. 关节连续性评分（与当前位置的距离）
        if current_joints is not None:
            joint_dist = np.linalg.norm(joints - current_joints)
            # 标准化：0-1（越小越好，取倒数）
            max_dist = np.linalg.norm(self.ub - self.lb)
            continuity_score = 1.0 - min(1.0, joint_dist / max_dist)
            scores.append(continuity_score * 0.4)  # 权重40%
        
        # 2. 任务空间精度评分
        valid, pos_err, rot_err = self.verify_pose(joints, target_pose)
        # 位置误差评分（0-1，越小越好）
        pos_score = 1.0 - min(1.0, pos_err / 0.1)  # 假设0.1m为最大容忍误差
        # 旋转误差评分（0-1，越小越好）
        rot_score = 1.0 - min(1.0, rot_err / 0.5)  # 假设0.5rad为最大容忍误差
        scores.append((pos_score * 0.7 + rot_score * 0.3) * 0.4)  # 权重40%
        
        # 3. 奇异性评分（最小奇异值）
        jac = self._compute_jacobian(joints)
        if jac is not None:
            _, s, _ = np.linalg.svd(jac)
            min_singular = np.min(s)
            # 归一化奇异值（0-1，越大越好）
            singularity_score = min(1.0, min_singular / 0.2)  # 假设0.2为良好阈值
            scores.append(singularity_score * 0.2)  # 权重20%
        
        return sum(scores) / len(scores)  # 加权平均
    
    def _is_out_of_bounds(self, joints):
        """检查关节是否超出限位"""
        return np.any(joints < self.lb) or np.any(joints > self.ub)

    def _compute_jacobian(self, joints, delta=1e-6, adaptive_delta=True):
        """数值计算雅可比矩阵"""
        if joints is None:
            return None
        if adaptive_delta:
            # 基于当前位置自动调整delta
            current_pose = self.fk(joints)
            pos_magnitude = np.linalg.norm(current_pose[:3,3])
            rot_magnitude = R.from_matrix(current_pose[:3,:3]).magnitude()
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

    def _is_valid(self, solution, tol, existing_solutions):
        """验证解的有效性"""
        if solution is None:
            return False

        # 检查关节限位
        if np.any(solution < self.lb) or np.any(solution > self.ub):
            return False

        # 检查解的唯一性
        for existing in existing_solutions:
            if np.linalg.norm(solution - existing) < tol:
                return False

        return True

    # def _select_best_solution(self, solutions, current_joints):
    #     """选择最优解：最接近当前位置或中间位置"""
    #     reference = current_joints if current_joints is not None else self.joint_mid
    #     return min(solutions, key=lambda sol: np.linalg.norm(sol - reference))
    
    def _select_best_solution(self, solutions, target_pose, current_joints):
        """综合关节空间距离和操作空间误差"""
        scores = []
        for sol in solutions:
            # 关节空间距离（标准化）
            joint_dist = np.linalg.norm(sol - current_joints) / len(sol)
            
            # 操作空间误差（标准化）
            _, pos_err, rot_err = self.verify_pose(sol, target_pose)
            task_err = pos_err * 0.7 + rot_err * 0.3  # 加权误差
            
            # 综合评分（越小越好）
            scores.append(0.6 * joint_dist + 0.4 * task_err)
        return solutions[np.argmin(scores)]

    def verify_pose(self, joints, target_pose, pos_tol=0.01, rot_tol=0.1):
        """
        验证关节角对应的实际位姿与目标位姿的误差

        :param joints: 关节角
        :param target_pose: 目标位姿(4x4齐次矩阵)
        :param pos_tol: 位置误差容差(m)
        :param rot_tol: 旋转误差容差(弧度)
        :return: (是否通过, 位置误差, 旋转误差)
        """
        if joints is None:
            return False, float("inf"), float("inf")

        actual_pose = self.fk(joints)
        if actual_pose is None:
            return False, float("inf"), float("inf")

        # 位置误差
        pos_error = np.linalg.norm(actual_pose[:3, 3] - target_pose[:3, 3])

        # 旋转误差
        r_target = R.from_matrix(target_pose[:3, :3])
        r_actual = R.from_matrix(actual_pose[:3, :3])
        rot_error = (r_target.inv() * r_actual).magnitude()

        return pos_error < pos_tol and rot_error < rot_tol, pos_error, rot_error

    def damped_least_squares_ik(self, target_pose, initial_joints, lambda_val=0.1, max_iter=50, tol=1e-4):
        """
        使用阻尼最小二乘法求解逆运动学
        :param target_pose: 目标位姿
        :param initial_joints: 初始关节角
        :param lambda_val: 阻尼系数
        :param max_iter: 最大迭代次数
        :param tol: 收敛容差
        :return: 关节角
        """
        if initial_joints is None:
            initial_joints = self.joint_mid.copy()

        joints = initial_joints.copy()

        for i in range(max_iter):
            # 计算当前位姿
            current_pose = self.fk(joints)
            if current_pose is None:
                break

            # 计算位姿误差
            pos_error = target_pose[:3, 3] - current_pose[:3, 3]
            r_current = R.from_matrix(current_pose[:3, :3])
            r_target = R.from_matrix(target_pose[:3, :3])
            rot_error = (r_target * r_current.inv()).as_rotvec()
            error = np.hstack([pos_error, rot_error])

            # 检查收敛
            if np.linalg.norm(error) < tol:
                break

            # 计算雅可比
            jac = self._compute_jacobian(joints)
            if jac is None:
                break

            # 计算阻尼最小二乘解
            jac_t = jac.T
            jjt = jac @ jac_t
            damping = lambda_val * lambda_val * np.eye(6)
            delta_theta = jac_t @ np.linalg.solve(jjt + damping, error)

            # 更新关节角
            joints += delta_theta
            joints = np.clip(joints, self.lb, self.ub)

        return joints

    def mat2quat_tf(self, matrix):
        """旋转矩阵转四元数（保持[w, x, y, z]格式）"""
        quat = R.from_matrix(matrix).as_quat()
        return [quat[3], quat[0], quat[1], quat[2]]

    def quat_to_rot_matrix(self, quat):
        """四元数转旋转矩阵"""
        w, x, y, z = quat
        return np.array(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ]
        )

    def interpolate_position(self, start_pos, end_pos, num_points):
        """线性插值位置"""
        return np.linspace(start_pos, end_pos, num_points)

    def generate_trajectory_by_dist(self, start_pos_, end_pos_, dist_=0.02):
        """生成轨迹"""
        distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
        num_points = int(distance / dist_)
        return self.interpolate_position(start_pos_, end_pos_, num_points)

    def create_pose(self, xyz_: list, quat_wxyz: list):
        ee_pose = np.eye(4)
        ee_pose[:3, 3] = np.array(xyz_)
        ee_pose[:3, :3] = self.quat_to_rot_matrix(quat_wxyz)
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
        quat_ = self.mat2quat_tf(rot_)
        return xyz_, rot_, quat_

    def inverse_kinematics(self, target_position, target_quaternion=None, initial_angles=None, orientation_weight=1.0, use_rotation_matrix=False):
        ee_pose = np.eye(4)
        ee_pose[:3, 3] = np.array(target_position)
        if target_quaternion is None:
            cuurent_xyz_, current_rot_, target_quaternion = self.forward_kinematics(target_position)
            ee_pose[:3, :3] = self.quat_to_rot_matrix(target_quaternion)
        else:
            ee_pose[:3, :3] = self.quat_to_rot_matrix(target_quaternion)
        if initial_angles is not None:
            self.current_joints = np.concatenate(([0], initial_angles))
        solution = self.move_to_pose(ee_pose, 20, 0.01)
        return solution[1:] if solution is not None else initial_angles

    def move_to_pose(self, target_pose, max_attempts=10, singularity_threshold=0.001):
        """
        移动到目标位姿

        :param target_pose: 目标末端位姿(4x4齐次矩阵)
        :param singularity_threshold: 奇异值检测阈值
        :return: (是否成功, 最终关节角, 位置误差, 旋转误差)
        """
        if self.current_joints is None:
            # 初始位置设为中间位置
            self.current_joints = self.joint_mid.copy()
        # 求解逆运动学
        solution = self.queue_based_ik(target_pose, current_joints=self.current_joints, max_attempts=max_attempts, singularity_threshold=singularity_threshold)

        if solution is None:
            print("IK failed to find valid solution! Trying singularity avoidance.")
            return self.avoid_singularity(target_pose, self.current_joints, singularity_threshold)

        # 更新当前关节
        self.current_joints = solution

        # 如果解接近奇异，尝试调整目标姿态
        if self.is_near_singularity(solution, singularity_threshold):
            print("Solution near singularity. Applying avoidance strategy.")
            return self.avoid_singularity(target_pose, solution, singularity_threshold)

        # 验证解的质量
        valid, pos_err, rot_err = self.verify_pose(solution, target_pose)
        if not valid:
            print(f"Solution verification failed! Position error: {pos_err:.4f}m, Rotation error: {rot_err:.4f}rad")
            return self.avoid_singularity(target_pose, solution, singularity_threshold)
        return solution

    def avoid_singularity(self, target_pose, current_joints, singularity_threshold=0.001):
        """奇异规避策略"""
        # 策略1：小幅调整目标位置
        print("Trying to adjust target pose...")
        adjustments = [
            lambda p: self._adjust_pose(p, [0, 0, 0.01]),  # Z+
            lambda p: self._adjust_pose(p, [0, 0, -0.01]),  # Z-
            lambda p: self._adjust_pose(p, [0.05, 0, 0]),  # X+
            lambda p: self._adjust_pose(p, [-0.05, 0, 0]),  # X-
            lambda p: self._adjust_pose(p, [0, 0.05, 0]),  # Y+
            lambda p: self._adjust_pose(p, [0, -0.05, 0]),  # Y-
        ]

        # 尝试不同调整
        for adjust in adjustments:
            adjusted_pose = adjust(target_pose)
            solution = self.robust_ik(adjusted_pose, current_joints=current_joints, max_attempts=5, singularity_threshold=singularity_threshold)

            if solution is not None and not self.is_near_singularity(solution, singularity_threshold):
                print("Singularity avoided with position adjustment.")
                # 验证并执行
                valid, pos_err, rot_err = self.verify_pose(solution, adjusted_pose)
                if valid:
                    return solution

        # 策略2：小幅调整目标方向
        print("Trying orientation adjustment...")
        r_target = R.from_matrix(target_pose[:3, :3])
        for angle in [0.2, -0.2, 0.3, -0.3]:  # 小角度调整
            r_adjusted = r_target * R.from_euler("z", angle)
            adjusted_pose = target_pose.copy()
            adjusted_pose[:3, :3] = r_adjusted.as_matrix()

            solution = self.robust_ik(adjusted_pose, current_joints=current_joints, max_attempts=5, singularity_threshold=singularity_threshold)

            if solution is not None and not self.is_near_singularity(solution, singularity_threshold):
                print(f"Singularity avoided with {angle} rad rotation adjustment.")
                # 验证并执行
                valid, pos_err, rot_err = self.verify_pose(solution, adjusted_pose)
                if valid:
                    return solution

        # 策略3：使用阻尼最小二乘法（伪逆）处理奇异点
        print("Using damped least squares approach for singularity handling.")
        solution = self.damped_least_squares_ik(target_pose, current_joints, lambda_val=0.01)  # 值越大越稳定但精度越低

        if solution is None:
            print("Damped least squares failed!")
            return current_joints

        # 验证并执行
        valid, pos_err, rot_err = self.verify_pose(solution, target_pose)
        if valid:
            return solution
        else:
            print("Damped solution verification failed.")
            return current_joints

    def _adjust_pose(self, pose, translation):
        """创建调整后的位姿"""
        adj_pose = pose.copy()
        adj_pose[:3, 3] += np.array(translation)
        return adj_pose

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
        # 避障扩展
        if self.obstacle_detected:
            bypass_point1 = mid1
            bypass_point2 = mid2
            control_points = np.vstack([start_pos, bypass_point1, bypass_point2, end_pos])
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
        for i in range(steps):
            all_points.append(positions[i])
            # all_quats.append(orientations[i])

        return all_points

    def visualize_trajectory(self, positions: np.array):
        """可视化笛卡尔空间轨迹"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        # 绘制轨迹
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], "bo-", linewidth=1, markersize=3, alpha=0.7, label="End-effector Path")

        # 添加起点和终点标记
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c="green", s=100, marker="o", edgecolors="k", label="Start")
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c="red", s=100, marker="*", edgecolors="k", label="End")

        # 添加坐标轴
        ax.quiver(0, 0, 0, 0.1, 0, 0, color="r", lw=2, alpha=0.5, label="X-axis")
        ax.quiver(0, 0, 0, 0, 0.1, 0, color="g", lw=2, alpha=0.5, label="Y-axis")
        ax.quiver(0, 0, 0, 0, 0, 0.1, color="b", lw=2, alpha=0.5, label="Z-axis")

        ax.set_xlabel("X (m)", fontsize=12)
        ax.set_ylabel("Y (m)", fontsize=12)
        ax.set_zlabel("Z (m)", fontsize=12)
        ax.set_title("End-effector Trajectory", fontsize=14)

        # 设置图例和网格
        ax.legend(fontsize=10, loc="upper left")
        ax.grid(True)

        # 设置等比例轴
        all_coords = np.concatenate([positions[:, 0], positions[:, 1], positions[:, 2]])
        min_val, max_val = np.min(all_coords), np.max(all_coords)
        range_val = max_val - min_val

        padding = range_val * 0.1  # 10% padding
        ax.set_xlim3d(min_val - padding, max_val + padding)
        ax.set_ylim3d(min_val - padding, max_val + padding)
        ax.set_zlim3d(min_val - padding, max_val + padding)

        plt.tight_layout()
        plt.savefig("./data/vis/trajectory_visualization.png", dpi=300)
        plt.show()


if __name__ == "__main__":
    arm_left_kinematics = ArmTracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link")
    arm_right_kinematics = ArmTracIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_r_link")
    # 设置目标位姿
    left_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    left_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    init_left_joints = None
    # 设置目标位姿
    right_pos = [0.32759669516187234, -0.1967146327303412, -0.07190695670671113]
    right_quat = [0.6549775332099196, 0.5350869754628191, -0.36644696956112155, -0.38781822827166285]
    init_right_joints = None
    while True:
        left_joints = arm_left_kinematics.inverse_kinematics(left_pos, left_quat, init_left_joints)
        right_joints = arm_right_kinematics.inverse_kinematics(right_pos, right_quat, init_right_joints)
        print("left_joints = ", list(left_joints))
        print("right_joints = ", list(right_joints))
    # 轨迹规划
    start_pose = arm_left_kinematics.create_pose(left_pos, left_quat)
    start_pose[:3, :3] = arm_left_kinematics.quat_to_rot_matrix(left_quat)

    end_pose = arm_left_kinematics.create_pose(right_pos, right_quat)
    points = arm_left_kinematics.plan(start_pose, end_pose, 20, True, [1.0, 0.0, 1.0])

    # 轨迹可视化
    arm_left_kinematics.visualize_trajectory(np.array(points))

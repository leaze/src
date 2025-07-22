import threading
import numpy as np
import concurrent.futures
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import make_interp_spline
from scipy.optimize import minimize
from tracikpy import TracIKSolver
import matplotlib.pyplot as plt
import transforms3d as tf3d
import time


class MySolver(TracIKSolver):
    def __init__(
        self,
        urdf_file,
        base_link,
        tip_link,
        is_left=True,
        timeout=0.005,
        epsilon=1e-5,
        solve_type="Speed",
    ):
        super().__init__(urdf_file, base_link, tip_link, timeout, epsilon, solve_type)
        self.ik_method = "SLSQP"
        self.lb, self.ub = self.joint_limits
        self.joint_limits_ = [(self.lb[i], self.ub[i]) for i in range(len(self.lb))][1:]
        self.lambda_damp = 0.01  # DLS阻尼因子初始值
        self.max_lambda = 0.1   # 最大阻尼因子
        self.singular_threshold = 1e-4  # 奇异值判定阈值
        self.solution_pool = []  # 解池

    def _compute_jacobian(self, joints, delta=1e-6, adaptive_delta=True):
        """数值计算雅可比矩阵"""
        if joints is None:
            return None
        if joints is not None and len(joints) == 7:
            joints = np.concatenate(([0], joints))
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

    # 新增：奇异值检测方法
    def is_singular(self, joint_angles):
        """检测是否处于奇异状态"""
        J = self._compute_jacobian(joint_angles)
        if J is None:
            return True
            
        # 计算雅可比矩阵的条件数
        cond = np.linalg.cond(J)
        # 计算最小奇异值
        _, s, _ = np.linalg.svd(J)
        min_singular = np.min(s)
        
        # 双指标奇异值检测
        return cond > 1e4 or min_singular < self.singular_threshold

    # 新增：DLS++修正方法
    def apply_dls_correction(self, joint_angles, target_position, target_quaternion):
        """应用DLS++修正奇异位置的解"""
        # 动态调整阻尼因子
        J = self._compute_jacobian(joint_angles)
        _, s, _ = np.linalg.svd(J)
        min_singular = np.min(s)
        
        # 根据奇异程度自适应调整阻尼因子
        lambda_adjusted = min(
            self.lambda_damp + (self.singular_threshold / min_singular) * 0.1,
            self.max_lambda
        )
        
        def dls_objective(current_angles):
            current_pos, current_rot, current_quat = self.forward_kinematics(current_angles)
            
            # 位置误差
            pos_error = np.linalg.norm(current_pos - target_position)
            
            # 方向误差
            ori_error = 0.0
            if target_quaternion is not None:
                dot = np.dot(target_quaternion, current_quat)
                dot = np.clip(dot, -1.0, 1.0)
                ori_error = 2 * np.arccos(np.abs(dot))
            
            # DLS修正项：加入阻尼因子的雅可比伪逆
            delta_theta = np.linalg.pinv(J.T @ J + lambda_adjusted**2 * np.eye(J.shape[1])) @ J.T
            damping_term = np.linalg.norm(delta_theta)
            
            return pos_error + ori_error + lambda_adjusted * damping_term
        
        # 约束优化
        result = minimize(
            dls_objective,
            joint_angles,
            method=self.ik_method,
            bounds=self.joint_limits_
        )
        
        return result.x if result.success else joint_angles
    
    # 重构：使用流程图策略的并行求解器
    def inverse_kinematics(self, target_position, target_quaternion=None, initial_angles=None):
        # 并行求解器 - 牛顿拉夫森法和SQP法
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # 任务1：牛顿拉夫森法（使用SLSQP优化器）
            nr_future = executor.submit(
                self._solve_with_optimizer,
                target_position,
                target_quaternion,
                initial_angles,
                "SLSQP"
            )
            
            # 任务2：SQP法（使用trust-constr优化器）
            sqp_future = executor.submit(
                self._solve_with_optimizer,
                target_position,
                target_quaternion,
                initial_angles,
                "trust-constr"
            )
            
            # 并行获取结果
            results = []
            for future in concurrent.futures.as_completed([nr_future, sqp_future]):
                if future.result() is not None:
                    results.append(future.result())
        
        # 选择最快且有效的解
        best_solution = None
        for result, solution in results:
            if solution is not None:
                best_solution = solution
                break
                
        # 没有有效解时的备选方案
        if best_solution is None:
            # 尝试使用父类方法
            q_init = initial_angles if initial_angles else [0] * len(self.joint_limits_)
            best_solution = super().get_ik(q_init, target_position, target_quaternion)
            if best_solution is None:
                # 随机生成初始关节角度作为最后手段
                best_solution = [np.random.uniform(low, high) for low, high in self.joint_limits_]
        
        # 奇异值检测和处理
        if self.is_singular(best_solution):
            # 应用DLS++修正
            corrected_solution = self.apply_dls_correction(
                best_solution, 
                target_position, 
                target_quaternion
            )
            
            # 加入解池
            self.solution_pool.append({
                'solution': corrected_solution,
                'joint_movement': np.linalg.norm(np.array(corrected_solution) - np.array(best_solution))
            })
            
            # 保留最近的10个解
            if len(self.solution_pool) > 10:
                self.solution_pool.pop(0)
                
            # 选择关节移动最小的解
            best_solution = min(
                self.solution_pool, 
                key=lambda x: x['joint_movement']
            )['solution']
        else:
            # 非奇异情况下，清空解池
            self.solution_pool = []
        
        return best_solution
    
    # 新增：供并行求解器使用的优化方法
    def _solve_with_optimizer(self, target_position, target_quaternion, initial_angles, method):
        """使用指定优化器求解IK"""
        start_time = time.time()
        
        # 处理初始角度
        if initial_angles is None:
            initial_angles = [0] * len(self.joint_limits_)
            
        # 优化目标函数
        def objective(joint_angles):
            pos_error, ori_error = self._calculate_pose_error(
                joint_angles, 
                target_position, 
                target_quaternion
            )
            penalty = self._joint_limit_penalty(joint_angles)
            return pos_error + ori_error + penalty
        
        # 使用指定优化方法求解
        result = minimize(
            objective, 
            initial_angles, 
            method=method,
            bounds=self.joint_limits_
        )
        
        # 计算耗时
        solve_time = time.time() - start_time
        
        # 返回结果和求解时间
        if result.success:
            return solve_time, result.x
        return solve_time, None
    
    # 重构：计算位姿误差（独立方法）
    def _calculate_pose_error(self, joint_angles, target_position, target_quaternion=None):
        """计算当前位置与目标位置之间的误差"""
        current_pos, _, current_quat = self.forward_kinematics(joint_angles)
        
        # 位置误差
        pos_error = np.linalg.norm(current_pos - target_position)
        
        # 方向误差
        ori_error = 0.0
        if target_quaternion is not None:
            current_quat_norm = self.normalize_quaternion(current_quat)
            dot = np.dot(target_quaternion, current_quat_norm)
            dot = np.clip(dot, -1.0, 1.0)
            ori_error = 2 * np.arccos(np.abs(dot))
            
        return pos_error, ori_error
    
    # 重构：关节限位惩罚（独立方法）
    def _joint_limit_penalty(self, joint_angles):
        """计算关节限位违反的惩罚项"""
        penalty = 0
        for i, angle in enumerate(joint_angles):
            low, high = self.joint_limits_[i]
            if angle < low:
                penalty += (low - angle) * 10.0
            elif angle > high:
                penalty += (angle - high) * 10.0
        return penalty
    
    # 保留以下方法...
    def normalize_quaternion(self, quat):
        """四元数归一化函数"""
        norm = np.linalg.norm(quat)
        return quat / norm if norm > 1e-10 else np.array([1.0, 0, 0, 0])

    def forward_kinematics(self, joint_angles):
        """正向运动学计算"""
        if joint_angles is not None and len(joint_angles) == 7:
            joint_angles = np.concatenate(([0], joint_angles))
        ee_out = super().fk(joint_angles)
        xyz_ = ee_out[:3, 3]
        rot_ = ee_out[:3, :3]
        quat_ = tf3d.quaternions.mat2quat(rot_)
        return xyz_, rot_, quat_

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
            mid1 = start_pos + (end_pos - start_pos) * 0.3 + np.array([0.01, -0.01, 0.01] * np.array(direction))  # 添加偏移控制点
            mid2 = start_pos + (end_pos - start_pos) * 0.7 + np.array([0.01, -0.01, 0.01] * np.array(direction))  # 添加偏移控制点

        control_points = np.vstack([start_pos, mid1, mid2, end_pos])
        # 避障扩展
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
            all_quats.append(orientations[i])

        return all_points, all_quats

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
    # 创建求解器
    arm_left_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True)
    arm_right_kinematics = MySolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_r_link", is_left=False)
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

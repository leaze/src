from scipy.spatial.transform import Rotation as R
from scipy.interpolate import make_interp_spline
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from tracikpy import TracIKSolver
import matplotlib.pyplot as plt
import transforms3d as tf3d
import numpy as np


class MySolver(TracIKSolver):
    def __init__(
        self,
        urdf_file,
        base_link,
        tip_link,
        is_left=True,  # 是否为左臂
        timeout=0.005,
        epsilon=1e-5,
        solve_type="Distance",
        max_attempts=10,
    ):
        super().__init__(urdf_file, base_link, tip_link, timeout, epsilon, solve_type)
        self.ik_method = "SLSQP"  # 优化方法: SLSQP, L-BFGS-B, trust-constr, Powell
        self.lb, self.ub = self.joint_limits  # 关节下限和上限
        self.joint_limits_ = [(self.lb[i], self.ub[i]) for i in range(len(self.lb))][1:]  # 转换为元组列表
        # self.joint_limits_ = [
        #     (-2.96, 2.96),  # shoulder_pitch
        #     (-3.4, 0.2618) if is_left else (-0.2618, 3.4),  # shoulder_roll
        #     (-2.96, 2.96),  # shoulder_yaw
        #     (-2.61, 0.261),  # elbow_pitch
        #     (-2.9671, 2.9671),  # elbow_yaw
        #     (-1.3, 1.65),  # wrist_pitch
        #     (-1.04, 0.785),  # wrist_roll
        # ]
        # self.joint_mid = (self.lb + self.ub) / 2.0
        # self.current_joints = [0.0] * self.number_of_joints
        # self.obstacle_detected = False
        # self.solution_queue = []  # 新增：求解结果队列
        # self.queue_size = 10  # 队列容量
        # self.max_attempts = max_attempts

    def normalize_quaternion(self, quat):
        """
        四元数归一化函数
        :param quat: 输入四元数 (list or np.array), 格式为 [w, x, y, z]
        :return: 归一化后的四元数 (np.array), 保证范数为1
        """
        norm = np.linalg.norm(quat)
        if norm < 1e-10:
            return np.array([1.0, 0, 0, 0])
        return quat / norm

    def forward_kinematics(self, joint_angles) -> np.ndarray:
        """
        :param joint_angles: 7维关节角
        :return: 3维xyz坐标, 旋转矩阵, 四元数
        """
        if joint_angles is not None and len(joint_angles) == 7:
            joint_angles = np.concatenate(([0], joint_angles))
        ee_out = super().fk(joint_angles)
        xyz_ = ee_out[:3, 3]
        rot_ = ee_out[:3, :3]
        # quat_ = self.mat2quat_tf(rot_)
        quat_ = tf3d.quaternions.mat2quat(rot_)
        return xyz_, rot_, quat_

    def inverse_kinematics(self, target_position, target_quaternion=None, initial_angles=None, orientation_weight=1.0, use_rotation_matrix=False):
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
            initial_angles = [0] * len(self.joint_limits_)

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
                    # target_rot = self.quat_to_rot_matrix(target_quaternion)  # wxyz顺序
                    target_rot = tf3d.quaternions.quat2mat(target_quaternion)  # wxyz顺序
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
                low, high = self.joint_limits_[i]
                if angle < low:
                    penalty += (low - angle) * 10.0
                elif angle > high:
                    penalty += (angle - high) * 10.0

            return total_error + penalty

        # 设置约束（关节限位）
        bounds = self.joint_limits_

        # 使用优化方法求解
        result = minimize(objective, initial_angles, method=self.ik_method, bounds=bounds)

        if result.success:
            return result.x
        else:
            # 如果优化失败，尝试使用不同的初始值
            best_result = result.x
            best_error = objective(result.x)

            for _ in range(5):
                new_initial = [np.random.uniform(low, high) for (low, high) in bounds]
                new_result = minimize(objective, new_initial, method=self.ik_method, bounds=bounds)
                if new_result.success:
                    return new_result.x
                new_error = objective(new_result.x)
                if new_error < best_error:
                    best_error = new_error
                    best_result = new_result.x

            # 返回最佳近似解
            return best_result
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
        # 正向运动学：计算末端位姿（位置和方向）
        valid_left_pos, valid_left_rot, valid_left_quat = arm_left_kinematics.forward_kinematics(left_joints)
        valid_right_pos, valid_right_rot, valid_right_quat = arm_right_kinematics.forward_kinematics(right_joints)
        valid_diff_xyz = np.linalg.norm(np.array(left_pos) - np.array(valid_left_pos))
        valid_diff_quat = np.linalg.norm(np.array(left_quat) - np.array(valid_left_quat))
        print(f"valid_diff_xyz = {valid_diff_xyz}, valid_diff_quat = {valid_diff_quat}")

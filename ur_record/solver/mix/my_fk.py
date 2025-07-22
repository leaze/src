from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
from tracikpy import TracIKSolver
import numpy as np

# import transforms3d as tf3d


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
        self.joint_limits_ = [(self.lb[i], self.ub[i]) for i in range(len(self.lb))]  # 转换为元组列表
        # self.joint_mid = (self.lb + self.ub) / 2.0
        # self.current_joints = [0.0] * self.number_of_joints
        # self.obstacle_detected = False
        # self.solution_queue = []  # 新增：求解结果队列
        # self.queue_size = 10  # 队列容量
        # self.max_attempts = max_attempts

    def mat2quat_tf(self, matrix):
        """旋转矩阵转四元数（保持[w, x, y, z]格式）"""
        quat = R.from_matrix(matrix).as_quat()
        return [quat[3], quat[0], quat[1], quat[2]]

    def quat_to_rot_matrix(self, quat):
        """四元数转旋转矩阵 (可选)"""
        w, x, y, z = quat
        return np.array(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ]
        )

    def normalize_quaternion(self, quat):
        """四元数归一化函数"""
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
        quat_ = self.mat2quat_tf(rot_)
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
    if True:
        left_joints = arm_left_kinematics.inverse_kinematics(left_pos, left_quat, init_left_joints)
        right_joints = arm_right_kinematics.inverse_kinematics(right_pos, right_quat, init_right_joints)
        print("left_joints = ", list(left_joints))
        print("right_joints = ", list(right_joints))

from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
from tracikpy import TracIKSolver
# import transforms3d as tf3d
import numpy as np

def quat_to_rot_matrix(quat_wxyz):
    """将[w, x, y, z]四元数转换为旋转矩阵"""
    return R.from_quat(quat_wxyz[[1, 2, 3, 0]]).as_matrix()


class StrictCoordinatedRobotIKSolver(TracIKSolver):
    def __init__(
        self,
        urdf_file,
        base_link,
        tip_link,
        is_left=True,  # 区分左右臂
        timeout=0.005,
        epsilon=1e-5,
        solve_type="Speed",
        max_attempts=10,
        orientation_weight=0.3,
        symmetry_weight=10.0,
        mirror_factors=None,
    ):
        """
        严格协调的双臂逆运动学求解器

        :param is_left: 是否为左臂
        :param symmetry_weight: 对称协调惩罚项的权重（设置为较大值强制对称）
        :param mirror_factors: 关节镜像因子列表
        """
        super().__init__(urdf_file, base_link, tip_link, timeout, epsilon, solve_type)
        self.max_attempts = max_attempts
        self.orientation_weight = orientation_weight
        self.symmetry_weight = symmetry_weight
        self.is_left = is_left
        self.initial_angles = [0.0, 0.15, 0.0, -0.0, 0.0, 0.0, 0.0] if self.is_left else [0.0, -0.15, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.lb, self.ub = self.joint_limits  # 关节下限和上限
        
        # 精确镜像因子（根据机器人结构定义）
        self.mirror_factors = (
            mirror_factors
            if mirror_factors
            else [
                1,  # waist (俯仰，不需要镜像)
                1,  # shoulder_pitch (俯仰，不需要镜像)
                -1,  # shoulder_roll (滚动，需要镜像)
                -1,  # shoulder_yaw (偏航，不需要镜像)
                1,  # elbow_pitch (俯仰，不需要镜像)
                -1,  # elbow_yaw (偏航，需要镜像)
                1,  # wrist_pitch (俯仰，不需要镜像)
                -1,  # wrist_roll (滚动，需要镜像)
            ]
        )

        # 参考关节角度（另一只手臂的关节角度）
        self.reference_joints = None
        # 镜像参考值（计算好的参考值的镜像）
        self.mirrored_reference = None

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
    
    def set_reference_joints(self, reference_joints):
        """设置参考关节角度（另一只手臂的关节角度）并计算其镜像值"""
        self.reference_joints = reference_joints
        self.mirrored_reference = self.mirror_joints(reference_joints)

    def mirror_joints(self, joints):
        """将关节角度转换为镜像值"""
        return [joint * factor for joint, factor in zip(joints, self.mirror_factors)]
    
    def mat2quat_tf(self, matrix):
        """旋转矩阵转四元数（保持[w, x, y, z]格式）"""
        quat = R.from_matrix(matrix).as_quat()
        return [quat[3], quat[0], quat[1], quat[2]]
    
    def interpolate_position(self, start_pos, end_pos, num_points):
        """线性插值位置"""
        return np.linspace(start_pos, end_pos, num_points)

    def generate_trajectory_by_dist(self, start_pos_, end_pos_, dist_=0.02):
        """生成轨迹"""
        distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
        num_points = int(distance / dist_)
        return self.interpolate_position(start_pos_, end_pos_, num_points)
    
    def ik(self, ee_pose, qinit=None, bx=1e-5, by=1e-5, bz=1e-5, brx=1e-3, bry=1e-3, brz=1e-3, enforce_mirror_constraint=False):
        """
        改进的逆运动学求解方法，强制镜像约束

        :param enforce_mirror_constraint: 是否强制镜像约束
        :return: 关节角度解或None
        """
        # 获取关节限位
        lb, ub = self.joint_limits

        # 准备尝试列表
        attempts_list = []

        # 如果有镜像参考值，优先使用它作为初始值
        if self.mirrored_reference is not None:
            attempts_list.append(self.mirrored_reference)

        # 如果有传入初始值
        if qinit is not None and len(qinit) == self.number_of_joints:
            attempts_list.append(qinit)
        if qinit is not None and len(qinit) == self.number_of_joints - 1:
            attempts_list.append([0.0] + list(qinit))

        # 生成随机初始值（在关节限位范围内）
        remaining_attempts = self.max_attempts - len(attempts_list)
        for _ in range(remaining_attempts):
            random_q = np.random.uniform(lb, ub)
            attempts_list.append(random_q)
        
        best_solution = None
        best_error = float("inf")

        # 尝试所有初始值
        for q_attempt in attempts_list:
            # 调用父类的IK求解
            solution = super().ik(ee_pose, q_attempt, bx, by, bz, brx, bry, brz)

            if solution is not None:
                # 计算基本误差（位置+方向）
                error = self.calculate_error(solution, ee_pose)

                # 添加关节限位惩罚
                penalty = self.joint_limits_penalty(solution, lb, ub)
                error += penalty

                # 添加镜像约束惩罚（强制启用）
                if self.mirrored_reference is not None:
                    symmetry_error = self.symmetry_penalty(solution)
                    error += self.symmetry_weight * symmetry_error

                # 更新最佳解
                if error < best_error:
                    best_solution = solution
                    best_error = error

        # 最终验证：如果强制约束，确保解与镜像参考值的差异在阈值内
        if best_solution is not None and enforce_mirror_constraint:
            if not self.validate_mirror_constraint(best_solution):
                return None  # 约束未满足，返回失败

        return best_solution if best_solution is not None else None

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
        ee_pose = np.eye(4)
        ee_pose[:3, 3] = np.array(target_position)
        if target_quaternion is None:
            cuurent_xyz_, current_rot_, target_quaternion = self.forward_kinematics(target_position)
            ee_pose[:3, :3] = self.quat_to_rot_matrix(target_quaternion)
        else:
            ee_pose[:3, :3] = self.quat_to_rot_matrix(target_quaternion)
        
        solution = self.ik(ee_pose, qinit=initial_angles)
        if solution is not None:
            return solution[1:]
        # 多次尝试（使用不同的随机初始值）
        for _ in range(self.max_attempts):
            qinit_random = np.random.uniform(self.lb, self.ub)
            solution = super().ik(ee_pose, qinit_random)
            if solution is not None:
                return solution

        # 如果所有尝试都失败，使用优化方法寻找最佳解
        return self.fallback_ik(ee_pose)
    
    def fallback_ik(self, ee_pose):
        """
        当所有标准IK求解失败时使用的备选方案
        """
        # 使用中值作为初始猜测
        initial_guess = (self.lb + self.ub) / 2.0

        # 定义目标函数（包含关节限位惩罚）
        def objective(joint_angles):
            # 计算当前位姿
            current_pose = self.fk(joint_angles)
            if current_pose is None:
                return float("inf")

            # 位置误差
            pos_error = np.linalg.norm(current_pose[:3, 3] - ee_pose[:3, 3])

            # 方向误差（使用四元数）
            q_current = self._q_from_pose(current_pose)
            q_target = self._q_from_pose(ee_pose)
            dot = np.dot(q_current, q_target)
            ori_error = np.arccos(2 * dot**2 - 1)

            # 关节限位惩罚
            penalty = 0
            for i, angle in enumerate(joint_angles):
                if angle < self.lb[i]:
                    penalty += (self.lb[i] - angle) * 10.0
                elif angle > self.ub[i]:
                    penalty += (angle - self.ub[i]) * 10.0

            return pos_error + ori_error + penalty

        # 使用优化求解
        result = minimize(objective, initial_guess, method="SLSQP", bounds=list(zip(self.lb, self.ub)))

        return result.x[1:] if result.success else initial_guess[1:]
    
    def calculate_error(self, joint_angles, target_pose):
        """计算位姿误差"""
        current_pose = self.fk(joint_angles)
        pos_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])

        # 使用四元数角度差（更精确）
        current_rot = current_pose[:3, :3]
        target_rot = target_pose[:3, :3]
        rot_error = self.rotation_error(current_rot, target_rot)

        return pos_error + self.orientation_weight * rot_error

    def rotation_error(self, R1, R2):
        """计算旋转矩阵之间的角度差（弧度）"""
        # R1 * R2^T 应该接近单位矩阵
        R_diff = np.dot(R1, R2.T)
        # 计算旋转角度
        cos_theta = (np.trace(R_diff) - 1) / 2
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        return np.abs(np.arccos(cos_theta))

    def joint_limits_penalty(self, joint_angles, lb, ub):
        """计算关节限位惩罚项"""
        penalty = 0.0
        for i, angle in enumerate(joint_angles):
            # 计算超出限位的程度
            violation_low = max(0.0, lb[i] - angle)
            violation_high = max(0.0, angle - ub[i])

            # 添加二次惩罚项
            penalty += 20.0 * (violation_low**2 + violation_high**2)
        return penalty

    def symmetry_penalty(self, solution):
        """计算镜像约束惩罚项"""
        if self.mirrored_reference is None:
            return 0.0

        # 计算解与镜像参考值的绝对差异
        diff = np.abs(np.array(solution) - np.array(self.mirrored_reference))
        return np.sum(diff**2)

    def validate_mirror_constraint(self, solution, threshold=0.05):
        """验证解是否满足镜像约束"""
        if self.mirrored_reference is None:
            return True

        diff = np.abs(np.array(solution) - np.array(self.mirrored_reference))
        max_diff = np.max(diff)
        return max_diff < threshold


if __name__ == "__main__":
    # 创建左臂求解器
    left_solver = StrictCoordinatedRobotIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True, symmetry_weight=10.0, max_attempts=15)  # 强约束

    # 创建右臂求解器
    right_solver = StrictCoordinatedRobotIKSolver("./ur_record/urdf/robot.urdf", "pelvis", "wrist_roll_r_link", is_left=False, symmetry_weight=10.0, max_attempts=15)  # 强约束

    left_xyz = np.array([0.32497879, 0.19681914, -0.06855335])
    left_wxyz = np.array([0.65497752, -0.53508699, -0.36644699, 0.38781821])
    left_mtrx = quat_to_rot_matrix(left_wxyz)
    # 构建4x4变换矩阵
    left_pose = np.eye(4)
    left_pose[:3, 3] = left_xyz
    left_pose[:3, :3] = left_mtrx

    right_xyz = np.array([0.32497879, -0.19681914, -0.06855335])
    right_wxyz = np.array([0.65497752, 0.53508699, -0.36644699, -0.38781821])
    right_mtrx = quat_to_rot_matrix(right_wxyz)
    # 构建4x4变换矩阵
    right_pose = np.eye(4)
    right_pose[:3, 3] = right_xyz
    right_pose[:3, :3] = right_mtrx

    # 优先求解左臂（作为参考）
    left_joints = left_solver.ik(left_pose)

    if left_joints is not None:
        print("左臂关节:", list(left_joints))

        # 设置右臂的参考关节（左臂关节的镜像）
        right_solver.set_reference_joints(left_joints)

        # 求解右臂，强制镜像约束
        right_joints = right_solver.ik(right_pose, enforce_mirror_constraint=True)
        if right_joints is not None:
            print("右臂关节:", list(right_joints))

            # 验证协同误差
            mirror_joints = right_solver.mirror_joints(left_joints)
            diff = np.abs(np.array(right_joints) - np.array(mirror_joints))
            max_diff = np.max(diff)
            avg_diff = np.mean(diff)

            print(f"最大关节误差: {max_diff:.6f} rad")
            print(f"平均关节误差: {avg_diff:.6f} rad")

            # 验证位姿精度
            left_fk = left_solver.fk(left_joints)
            right_fk = right_solver.fk(right_joints)
            pos_error = np.linalg.norm(right_fk[:3, 3] - right_xyz)
            rot_error = right_solver.rotation_error(right_fk[:3, :3], quat_to_rot_matrix(right_wxyz))

            print(f"右臂位置误差: {pos_error:.6f} m")
            print(f"右臂方向误差: {rot_error:.6f} rad")
        else:
            print("右臂求解失败（镜像约束未满足）")
    else:
        print("左臂求解失败")

#!/usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation as R
from tracikpy import TracIKSolver
from scipy.spatial.transform import Slerp

def quat_to_rot_matrix(quat):
    """四元数转旋转矩阵"""
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ]
    )

class RobustTracIKSolver:
    """增强版TRAC-IK求解器，包含奇异值检测与规避"""
    def __init__(self, base_solver: TracIKSolver):
        """
        使用已有的TracIKSolver实例初始化
        
        :param base_solver: 基础TracIKSolver实例
        """
        self.base_solver = base_solver
        
        # 缓存关节限位信息
        self.lb, self.ub = self.base_solver.joint_limits
        self.lb = np.array(self.lb)
        self.ub = np.array(self.ub)
        self.joint_mid = (self.lb + self.ub) / 2.0
        
        # 委托属性访问
        self.urdf_string = base_solver._urdf_string
        self.base_link = base_solver.base_link
        self.tip_link = base_solver.tip_link
        self._timeout = base_solver._timeout
        self._epsilon = base_solver._epsilon
        self._solve_type = base_solver._solve_type
        self.number_of_joints = base_solver.number_of_joints
        self.joint_names = base_solver.joint_names
        self.link_names = base_solver.link_names

    def robust_ik(self, ee_pose, current_joints=None, max_attempts=10, tol=1e-3, singularity_threshold=0.1):
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
            solution = self.base_solver.ik(ee_pose, qinit=current_joints)
            if self._is_valid(solution, tol, valid_solutions):
                valid_solutions.append(solution)
        
        # 尝试使用中间位置作为种子
        if not valid_solutions:
            solution = self.base_solver.ik(ee_pose, qinit=self.joint_mid)
            if self._is_valid(solution, tol, valid_solutions):
                valid_solutions.append(solution)
        
        # 随机种子尝试
        attempt_count = 0
        while len(valid_solutions) < 3 and attempt_count < max_attempts:
            qinit = np.random.default_rng().uniform(self.lb, self.ub)
            solution = self.base_solver.ik(ee_pose, qinit=qinit)
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
        
        return self._select_best_solution(non_singular_solutions, current_joints)

    def is_near_singularity(self, joints, threshold=0.1):
        """
        检测是否接近奇异位形
        :param joints: 当前关节角
        :param threshold: 奇异值检测阈值（最小奇异值）
        :return: 布尔值，表示是否接近奇异
        """
        if joints is None:
            return False
            
        jac = self._compute_jacobian(joints)
        if jac is None:
            return False
            
        _, s, _ = np.linalg.svd(jac)
        min_singular = np.min(s)
        return min_singular < threshold

    def _compute_jacobian(self, joints, delta=1e-6):
        """数值计算雅可比矩阵"""
        if joints is None:
            return None
            
        jac = np.zeros((6, self.number_of_joints))
        current_pose = self.base_solver.fk(joints)
        
        if current_pose is None:
            return None
            
        for i in range(self.number_of_joints):
            # 扰动第i个关节
            perturbed_joints = joints.copy()
            perturbed_joints[i] += delta
            
            # 计算扰动后的位姿
            perturbed_pose = self.base_solver.fk(perturbed_joints)
            if perturbed_pose is None:
                continue
                
            # 计算位置差异
            pos_diff = (perturbed_pose[:3, 3] - current_pose[:3, 3]) / delta
            
            # 计算姿态差异（角度轴表示）
            r_current = R.from_matrix(current_pose[:3, :3])
            r_perturbed = R.from_matrix(perturbed_pose[:3, :3])
            rot_diff = (r_perturbed * r_current.inv()).as_rotvec() / delta
            
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

    def _select_best_solution(self, solutions, current_joints):
        """选择最优解：最接近当前位置或中间位置"""
        reference = current_joints if current_joints is not None else self.joint_mid
        return min(solutions, key=lambda sol: np.linalg.norm(sol - reference))

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
            return False, float('inf'), float('inf')
            
        actual_pose = self.base_solver.fk(joints)
        if actual_pose is None:
            return False, float('inf'), float('inf')
            
        # 位置误差
        pos_error = np.linalg.norm(actual_pose[:3, 3] - target_pose[:3, 3])
        
        # 旋转误差
        r_target = R.from_matrix(target_pose[:3, :3])
        r_actual = R.from_matrix(actual_pose[:3, :3])
        rot_error = (r_target.inv() * r_actual).magnitude()
        
        return pos_error < pos_tol and rot_error < rot_tol, pos_error, rot_error

    # 委托所有未实现的方法到基础求解器
    def __getattr__(self, name):
        """将未定义的属性访问委托给基础求解器"""
        return getattr(self.base_solver, name)
    
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
            current_pose = self.base_solver.fk(joints)
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


class RobotController:
    """机器人控制封装类，结合运动学求解和执行监控"""
    def __init__(self, ik_solver: RobustTracIKSolver):
        """
        使用增强的IK求解器初始化
        
        :param ik_solver: RobustTracIKSolver实例
        """
        self.ik_solver = ik_solver
        self.current_joints = None
    def quat_to_rot_matrix(self, quat):
        """四元数转旋转矩阵"""
        w, x, y, z = quat
        return np.array(
            [
                [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
            ]
        )
    def update_joint_limits(self, new_limits):
        """
        更新关节限位（考虑可能的硬件调整）
        
        :param new_limits: (lower_bounds, upper_bounds)元组
        """
        self.ik_solver.base_solver.joint_limits = new_limits
        # 更新缓存
        self.ik_solver.lb, self.ik_solver.ub = self.ik_solver.base_solver.joint_limits
        self.ik_solver.joint_mid = (self.ik_solver.lb + self.ik_solver.ub) / 2.0
        
    def move_to_pose(self, target_pose, max_attempts=10, singularity_threshold=0.01):
        """
        移动到目标位姿
        
        :param target_pose: 目标末端位姿(4x4齐次矩阵)
        :param singularity_threshold: 奇异值检测阈值
        :return: (是否成功, 最终关节角, 位置误差, 旋转误差)
        """
        if self.current_joints is None:
            # 初始位置设为中间位置
            self.current_joints = self.ik_solver.joint_mid.copy()
            
        # 求解逆运动学
        solution = self.ik_solver.robust_ik(
            target_pose, 
            current_joints=self.current_joints,
            max_attempts=max_attempts,
            singularity_threshold=singularity_threshold
        )
        
        if solution is None:
            print("IK failed to find valid solution! Trying singularity avoidance.")
            return self.avoid_singularity(target_pose, self.current_joints)
        
        # 如果解接近奇异，尝试调整目标姿态
        if self.ik_solver.is_near_singularity(solution, singularity_threshold):
            print("Solution near singularity. Applying avoidance strategy.")
            return self.avoid_singularity(target_pose, solution)
        
        return solution
        # 验证解的质量
        valid, pos_err, rot_err = self.ik_solver.verify_pose(solution, target_pose)
        if not valid:
            print(f"Solution verification failed! Position error: {pos_err:.4f}m, Rotation error: {rot_err:.4f}rad")
            return self.avoid_singularity(target_pose, solution)
        
        # 执行运动（模拟）
        executed_joints = self._execute_motion(solution)
        
        # 更新当前关节位置
        self.current_joints = executed_joints
        
        # 验证最终位姿
        final_valid, final_pos_err, final_rot_err = self.ik_solver.verify_pose(
            executed_joints, target_pose
        )
        
        return final_valid, executed_joints, final_pos_err, final_rot_err
    
    def avoid_singularity(self, target_pose, current_joints, singularity_threshold=0.1):
        """奇异规避策略"""
        # 策略1：小幅调整目标位置
        adjustments = [
            lambda p: self._adjust_pose(p, [0, 0, 0.02]),  # Z+
            lambda p: self._adjust_pose(p, [0, 0, -0.02]),  # Z-
            lambda p: self._adjust_pose(p, [0.02, 0, 0]),   # X+
            lambda p: self._adjust_pose(p, [-0.02, 0, 0]),  # X-
            lambda p: self._adjust_pose(p, [0, 0.02, 0]),   # Y+
            lambda p: self._adjust_pose(p, [0, -0.02, 0])  # Y-
        ]
        
        # 尝试不同调整
        for adjust in adjustments:
            adjusted_pose = adjust(target_pose)
            solution = self.ik_solver.robust_ik(
                adjusted_pose,
                current_joints=current_joints,
                max_attempts=5,
                singularity_threshold=singularity_threshold
            )
            
            if solution is not None and not self.ik_solver.is_near_singularity(solution, singularity_threshold):
                print("Singularity avoided with position adjustment.")
                # 验证并执行
                valid, pos_err, rot_err = self.ik_solver.verify_pose(solution, adjusted_pose)
                if valid:
                    executed_joints = self._execute_motion(solution)
                    self.current_joints = executed_joints
                    return True, executed_joints, pos_err, rot_err
        
        # 策略2：小幅调整目标方向
        print("Trying orientation adjustment...")
        r_target = R.from_matrix(target_pose[:3, :3])
        for angle in [0.1, -0.1, 0.2, -0.2]:  # 小角度调整
            r_adjusted = r_target * R.from_euler('z', angle)
            adjusted_pose = target_pose.copy()
            adjusted_pose[:3, :3] = r_adjusted.as_matrix()
            
            solution = self.ik_solver.robust_ik(
                adjusted_pose,
                current_joints=current_joints,
                max_attempts=5,
                singularity_threshold=singularity_threshold
            )
            
            if solution is not None and not self.ik_solver.is_near_singularity(solution, singularity_threshold):
                print(f"Singularity avoided with {angle} rad rotation adjustment.")
                # 验证并执行
                valid, pos_err, rot_err = self.ik_solver.verify_pose(solution, adjusted_pose)
                if valid:
                    executed_joints = self._execute_motion(solution)
                    self.current_joints = executed_joints
                    return True, executed_joints, pos_err, rot_err
        
        # 策略3：使用阻尼最小二乘法（伪逆）处理奇异点
        print("Using damped least squares approach for singularity handling.")
        solution = self.ik_solver.damped_least_squares_ik(
            target_pose, 
            current_joints,
            lambda_val=0.2
        )
        
        if solution is None:
            print("Damped least squares failed!")
            return False, None, float('inf'), float('inf')
        
        # 验证并执行
        valid, pos_err, rot_err = self.ik_solver.verify_pose(solution, target_pose)
        if valid:
            executed_joints = self._execute_motion(solution)
            self.current_joints = executed_joints
            return True, executed_joints, pos_err, rot_err
        else:
            print("Damped solution verification failed.")
            return False, solution, pos_err, rot_err
    
    def _execute_motion(self, target_joints, max_speed=0.5):
        """
        模拟执行关节运动（真实机器人应调用底层API）
        
        :param target_joints: 目标关节角
        :param max_speed: 最大关节速度(rad/s)
        :return: 实际执行的关节角（考虑物理限位）
        """
        # 模拟实际执行中的限位约束
        executed_joints = np.clip(target_joints, self.ik_solver.lb, self.ik_solver.ub)
        
        # 模拟运动时间计算（基于最大关节速度）
        if self.current_joints is not None:
            delta = np.abs(executed_joints - self.current_joints)
            max_delta = np.max(delta)
            duration = max_delta / max_speed
            print(f"Executing motion in {duration:.2f}s to joints: {list(executed_joints[1:])}")
        else:
            print(f"Executing motion to joints: {list(executed_joints[1:])}")
            
        return executed_joints
    
    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        if self.current_joints is None:
            return None
        return self.ik_solver.fk(self.current_joints)
    
    def _adjust_pose(self, pose, translation):
        """创建调整后的位姿"""
        adj_pose = pose.copy()
        adj_pose[:3, 3] += np.array(translation)
        return adj_pose

    def plan_path(self, start_pose, end_pose, steps=10):
        """生成笛卡尔空间路径"""
        waypoints = []
        for t in np.linspace(0, 1, steps):
            # 线性插值位置
            pos = (1-t)*start_pose[:3, 3] + t*end_pose[:3, 3]
            
            # 球面线性插值旋转
            r_start = R.from_matrix(start_pose[:3, :3])
            r_end = R.from_matrix(end_pose[:3, :3])
            r_interp = r_start.slerp(r_end, t)
            
            pose = np.eye(4)
            pose[:3, 3] = pos
            pose[:3, :3] = r_interp.as_matrix()
            waypoints.append(pose)
        return waypoints
    
    def joint_space_interpolation(self, start_joints, end_joints, steps=10):
        """关节空间插值"""
        trajectories = []
        for i in range(len(start_joints)):
            traj = np.linspace(start_joints[i], end_joints[i], steps)
            trajectories.append(traj)
        return np.array(trajectories).T
    
    def mat2quat_tf(self, matrix):
        """旋转矩阵转四元数（保持[w, x, y, z]格式）"""
        quat = R.from_matrix(matrix).as_quat()
        return [quat[3], quat[0], quat[1], quat[2]]
    
    def forward_kinematics(self, joint_angles):
        """
        :param joint_angles: 7维关节角
        :return: 3维xyz坐标, 旋转矩阵, 四元数
        """
        if joint_angles is not None and len(joint_angles) == 7:
            joint_angles = np.concatenate(([0], joint_angles))
        ee_out = self.ik_solver.base_solver.fk(joint_angles)
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
        solution = self.move_to_pose(ee_pose, )
        if solution is not None:
            return solution[1:]
        return solution[1:]
    
    def interpolate_position(self, start_pos, end_pos, num_points):
        """线性插值位置"""
        return np.linspace(start_pos, end_pos, num_points)

    def generate_trajectory_by_dist(self, start_pos_, end_pos_, dist_=0.02):
        """生成轨迹"""
        distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
        num_points = int(distance / dist_)
        return self.interpolate_position(start_pos_, end_pos_, num_points)
        
# ------------------------------
# 使用示例
# ------------------------------
if __name__ == "__main__":
    # 1. 创建基础求解器
    left_solver = TracIKSolver(urdf_file="./ur_record/urdf/robot.urdf", base_link="pelvis", tip_link="wrist_roll_l_link")
    right_solver = TracIKSolver(urdf_file="./ur_record/urdf/robot.urdf", base_link="pelvis", tip_link="wrist_roll_r_link")
    
    # 2. 创建增强求解器
    arm_left_kinematics = RobustTracIKSolver(left_solver)
    arm_right_kinematics = RobustTracIKSolver(right_solver)
    
    # 3. 创建控制器
    left_controller = RobotController(arm_left_kinematics)
    right_controller = RobotController(arm_right_kinematics)
    
    # 设置目标位姿
    left_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    left_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]

    left_target_pose = np.eye(4)
    left_target_pose[:3, 3] = np.array(left_pos)
    left_target_pose[:3, :3] = quat_to_rot_matrix(left_quat)
    
    # 设置目标位姿
    right_pos = [0.32759669516187234, -0.1967146327303412, -0.07190695670671113]
    right_quat = [0.6549775332099196, 0.5350869754628191, -0.36644696956112155, -0.38781822827166285]
    right_target_pose = np.eye(4)
    right_target_pose[:3, 3] = np.array(right_pos)
    right_target_pose[:3, :3] = quat_to_rot_matrix(right_quat)
    
    # 执行左臂运动
    left_joints = left_controller.move_to_pose(
        left_target_pose,
        singularity_threshold=0.01
    )
    print("left_joints", list(left_joints))

    # 执行右臂运动
    right_joints = right_controller.move_to_pose(
        right_target_pose,
        singularity_threshold=0.01
    )
    print("right_joints", list(right_joints))
from coordinated_solver import StrictCoordinatedRobotIKSolver, quat_to_rot_matrix
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import numpy as np

class CoordinatedIKSolver():
    def __init__(self):
        # 创建左臂求解器
        self.left_solver = StrictCoordinatedRobotIKSolver("./description/tiangong_description/urdf/robot.urdf", "pelvis", "wrist_roll_l_link", is_left=True, symmetry_weight=10.0, max_attempts=15)  # 强约束
        # 创建右臂求解器
        self.right_solver = StrictCoordinatedRobotIKSolver("./description/tiangong_description/urdf/robot.urdf", "pelvis", "wrist_roll_r_link", is_left=False, symmetry_weight=10.0, max_attempts=15)  # 强约束
    

    def ik_dual(self, left_target_position, left_target_quaternion, left_initial_angles, right_target_position, right_target_quaternion, right_initial_angles):
        left_xyz = np.array(left_target_position)
        left_wxyz = np.array(left_target_quaternion)
        left_mtrx = quat_to_rot_matrix(left_wxyz)
        # 构建4x4变换矩阵
        left_pose = np.eye(4)
        left_pose[:3, 3] = left_xyz
        left_pose[:3, :3] = left_mtrx

        right_xyz = np.array(right_target_position)
        right_wxyz = np.array(right_target_quaternion)
        right_mtrx = quat_to_rot_matrix(right_wxyz)
        # 构建4x4变换矩阵
        right_pose = np.eye(4)
        right_pose[:3, 3] = right_xyz
        right_pose[:3, :3] = right_mtrx

        # 优先求解左臂（作为参考）
        left_joints = self.left_solver.ik(left_pose, qinit=left_initial_angles)

        if left_joints is not None:
            # 设置右臂的参考关节（左臂关节的镜像）
            self.right_solver.set_reference_joints(left_joints)

            # 求解右臂，强制镜像约束
            right_joints = self.right_solver.ik(right_pose, qinit=right_initial_angles, enforce_mirror_constraint=True)
            if right_joints is not None:

                # 验证协同误差
                mirror_joints = self.right_solver.mirror_joints(left_joints)
                diff = np.abs(np.array(right_joints) - np.array(mirror_joints))
                max_diff = np.max(diff)
                avg_diff = np.mean(diff)

                # print(f"最大关节误差: {max_diff:.6f} rad")
                # print(f"平均关节误差: {avg_diff:.6f} rad")

                return left_joints[1:], right_joints[1:]
            else:
                print("右臂求解失败（镜像约束未满足）")
                return left_joints[1:], right_joints[1:]
        else:
            print("左臂求解失败")
            return left_joints[1:], right_joints[1:]
    
    def interpolate_position(self, start_pos, end_pos, num_points):
        """线性插值位置"""
        return np.linspace(start_pos, end_pos, num_points)

    def generate_trajectory_by_dist(self, start_pos_, end_pos_, dist_=0.02):
        """生成轨迹"""
        distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
        num_points = int(distance / dist_)
        return self.interpolate_position(start_pos_, end_pos_, num_points)
    
    @staticmethod
    def interpolate_orientation(start_quat, end_quat, num_points):
        """球面线性插值四元数姿态"""
        if num_points < 2:
            return [start_quat] if num_points == 1 else []
            
        # 创建旋转对象
        rots = R.from_quat([start_quat, end_quat])
        
        # 创建球面线性插值器
        slerp = Slerp([0, 1], rots)
        
        # 生成插值点
        times = np.linspace(0, 1, num_points)
        interp_rots = slerp(times)
        
        # 转换回四元数 (wxyz顺序)
        return [R.as_quat(rot)[[3, 0, 1, 2]] for rot in interp_rots]
    
    # def get_trajectory_points(self, left_start_pos, left_end_pos, right_start_pos, right_end_pos, dist_=0.02):
    #     # 计算距离
    #     left_distance = np.linalg.norm(np.array(left_start_pos) - np.array(left_end_pos))
    #     right_distance = np.linalg.norm(np.array(right_start_pos) - np.array(right_end_pos))
        
    #     # 计算点数，确保至少有一个点
    #     left_num_points = max(int(left_distance / dist_), 1)
    #     right_num_points = max(int(right_distance / dist_), 1)
        
    #     # 取最大，防止轨迹不同步
    #     num_points = max(left_num_points, right_num_points)
        
    #     # 线性插值轨迹，确保点数一致
    #     left_pose_ = np.linspace(np.array(left_start_pos), np.array(left_end_pos), num_points)
    #     right_pose_ = np.linspace(np.array(right_start_pos), np.array(right_end_pos), num_points)
        
    #     return list(left_pose_), list(right_pose_)
    
    def get_trajectory_points(self, left_start_pos, left_start_quat, left_end_pos, left_end_quat,
                             right_start_pos, right_start_quat, right_end_pos, right_end_quat,
                             max_step=0.02):
        """
        生成同步的双臂轨迹点
        返回: 
        (left_positions, left_orientations, right_positions, right_orientations)
        """
        # 计算左右臂的距离
        left_dist = np.linalg.norm(np.array(left_end_pos) - np.array(left_start_pos))
        right_dist = np.linalg.norm(np.array(right_end_pos) - np.array(right_start_pos))
        
        # 统一轨迹点数 (基于最长路径)
        num_points = max(
            int(np.ceil(left_dist / max_step)) + 1,
            int(np.ceil(right_dist / max_step)) + 1,
            2  # 至少包含起点和终点
        )
        
        # 线性插值位置
        left_positions = np.linspace(left_start_pos, left_end_pos, num_points)
        right_positions = np.linspace(right_start_pos, right_end_pos, num_points)
        
        # 球面线性插值姿态
        left_orientations = self.interpolate_orientation(left_start_quat, left_end_quat, num_points)
        right_orientations = self.interpolate_orientation(right_start_quat, right_end_quat, num_points)
        
        # 保持一致的姿态插值
        if not left_orientations:
            left_orientations = [left_start_quat] * num_points
        if not right_orientations:
            right_orientations = [right_start_quat] * num_points
        
        # 验证轨迹点数量一致性
        assert len(left_positions) == len(right_positions) == \
               len(left_orientations) == len(right_orientations), \
               "轨迹点数量不一致"
        
        return left_positions, left_orientations, right_positions, right_orientations
    

if __name__ == "__main__":
    coordinat_ik = CoordinatedIKSolver()
    left_pos = [0.32497879, 0.19681914, -0.06855335]
    left_end = [0.15497879, 0.19681914, -0.01855335]
    left_quat = [0.65497752, -0.53508699, -0.36644699, 0.38781821]
    left_init = [-0.504333764830546, -0.19203258438311485, 0.5687797544031549, -0.3358517591248969, 0.6184368370260153, 0.33441139366286493, -0.7184362322649265]

    right_pos = [0.32497879, -0.19681914, -0.06855335]
    right_end = [0.15497879, -0.19681914, -0.01855335]
    right_quat = [0.65497752, 0.53508699, -0.36644699, -0.38781821]
    right_init = [-0.5061886639850919, 0.19123308433375877, -0.569791921351373, -0.33175675879944155, -0.6171442084783899, 0.33267085294472976, 0.7196996180504094]

    left_joints, right_joints = coordinat_ik.ik_dual(left_pos, left_quat, left_init, right_pos, right_quat, right_init)
    print("left_joints = ", list(left_joints))
    print("right_joints = ", list(right_joints))
    
    left_pose_ = coordinat_ik.generate_trajectory_by_dist(left_pos, left_end)
    right_pose_ = coordinat_ik.generate_trajectory_by_dist(right_pos, right_end)
    # left_pose_, right_pose_ = coordinat_ik.get_trajectory_points(left_pos, left_end, right_pos, right_end)
    left_pose_, left_qaut_, right_pose_, right_quat_ = coordinat_ik.get_trajectory_points(left_pos, left_quat, left_end, left_quat, right_pos, right_quat, right_end, right_quat)
    print("left_pose_ = ", left_pose_)
    print("right_pose_ = ", right_pose_)
    
    for i in range(len(left_pose_)):
        left_pos = left_pose_[i]
        right_pos = right_pose_[i]
        # print("left_pos = ", list(left_pos))
        # print("right_pos = ", list(right_pos))
        left_joints, right_joints = coordinat_ik.ik_dual(left_pos, left_quat, left_init, right_pos, right_quat, right_init)
        # 更新初始位置
        left_init = left_joints
        right_init = right_joints
        # 验证误差
        print("left_joints = ", list(left_joints))
        print("right_joints = ", list(right_joints))
        fk_left_xyz, _, fk_left_quat = coordinat_ik.left_solver.forward_kinematics(left_joints)
        fk_right_xyz, _, fk_right_quat = coordinat_ik.right_solver.forward_kinematics(right_joints)
        print("left_fk_xyz = ", fk_left_xyz)
        print("right_fk_xyz = ", fk_right_xyz)
        # print("left_diff = ", left_pos - fk_left_xyz)
        # print("right_diff = ", right_pos - fk_right_xyz)
        print("==============================================================================================================================================================")
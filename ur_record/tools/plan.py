import numpy as np
from scipy.interpolate import make_interp_spline
from geometry_msgs.msg import Pose  # 假设使用ROS消息类型
from visual import visualize_trajectory
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R

class TrajectoryPlanner:
    def __init__(self):
        pass

    def plan_path(self, start_pose: Pose, end_pose: Pose, steps=10):
        """
        生成起点到终点的非直线轨迹(三维B样条)
        
        参数:
            start_pose: 起点位姿 (geometry_msgs/Pose)
            end_pose: 终点位姿 (geometry_msgs/Pose)
            steps: 路径点数量（包括起点终点）
            
        返回:
            path: 轨迹点位姿列表 (list of geometry_msgs/Pose)
        """
        # 提取起点终点坐标
        start_pos = np.array([start_pose.position.x, 
                              start_pose.position.y, 
                              start_pose.position.z])
        end_pos = np.array([end_pose.position.x,
                            end_pose.position.y,
                            end_pose.position.z])
        
        # 生成曲形控制点 (在起点终点之间添加偏移控制点)
        mid1 = start_pos + (end_pos - start_pos) * 0.3 + np.random.uniform(-0.05, 0.05, 3)
        mid2 = start_pos + (end_pos - start_pos) * 0.7 + np.random.uniform(-0.05, 0.05, 3)
        # 指定特定偏移量（非随机）
        # mid1 = start_pos + (end_pos - start_pos) * 0.3 + np.array([0.1, -0.1, 0.1])
        # mid2 = start_pos + (end_pos - start_pos) * 0.7 + np.array([0.1, -0.1, 0.1])
        
        control_points = np.vstack([
            start_pos,
            mid1,
            mid2,
            end_pos
        ])
        
        # 参数化控制点
        t = [0, 0.3, 0.7, 1.0]
        spline = make_interp_spline(t, control_points, k=3)
        
        # 生成平滑轨迹
        u = np.linspace(0, 1, steps)
        positions = spline(u)
        
        # 姿态插值（四元数球面线性插值）
        start_quat = self.quaternion_to_array(start_pose.orientation)
        end_quat = self.quaternion_to_array(end_pose.orientation)
        orientations = self.slerp_orientations(start_quat, end_quat, steps)
        
        # 构建路径点
        path = []
        all_points = []
        for i in range(steps):
            pose = Pose()
            pose.position.x = positions[i, 0]
            pose.position.y = positions[i, 1]
            pose.position.z = positions[i, 2]
            pose.orientation = self.array_to_quaternion(orientations[i])
            path.append(pose)
            all_points.append(positions[i])
            
        return path, all_points
    
    def plan(self, start_pose, end_pose, steps=10):
        """
        生成起点到终点的非直线轨迹(三维B样条)
        
        参数:
            start_pose: 起点位姿 (geometry_msgs/Pose)
            end_pose: 终点位姿 (geometry_msgs/Pose)
            steps: 路径点数量（包括起点终点）
            
        返回:
            path: 轨迹点位姿列表 (list of geometry_msgs/Pose)
        """
        # 提取起点终点坐标
        start_pos = start_pose[:3, 3]
        end_pos = end_pose[:3, 3]
        r_start = R.from_matrix(start_pose[:3, :3])
        r_end = R.from_matrix(end_pose[:3, :3])
        # 生成曲形控制点 (在起点终点之间添加偏移控制点)
        mid1 = start_pos + (end_pos - start_pos) * 0.3 + np.random.uniform(-0.01, 0.01, 3)
        mid2 = start_pos + (end_pos - start_pos) * 0.7 + np.random.uniform(-0.01, 0.01, 3)
        # 指定特定偏移量（非随机）
        mid1 = start_pos + (end_pos - start_pos) * 0.3 + np.array([0.01, -0.00, 0.01])
        mid2 = start_pos + (end_pos - start_pos) * 0.7 + np.array([0.01, -0.00, 0.01])
        
        control_points = np.vstack([
            start_pos,
            mid1,
            mid2,
            end_pos
        ])
        
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
        path = []
        all_points = []
        for i in range(steps):
            pose = Pose()
            pose.position.x = positions[i, 0]
            pose.position.y = positions[i, 1]
            pose.position.z = positions[i, 2]
            pose.orientation = self.array_to_quaternion(orientations[i])
            path.append(pose)
            all_points.append(positions[i])
            
        return path, all_points
    
    def quaternion_to_array(self, quat: Quaternion):
        return np.array([quat.x, quat.y, quat.z, quat.w])
    
    def array_to_quaternion(self, arr):
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=arr[0], y=arr[1], z=arr[2], w=arr[3])
    
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

# 使用示例
if __name__ == "__main__":
    planner = TrajectoryPlanner()
    
    # 创建示例位姿
    start = Pose()
    start.position.x, start.position.y, start.position.z = 0, 0, 0
    start.orientation.w = 1  # 单位四元数
    
    end = Pose()
    end.position.x, end.position.y, end.position.z = 1, 1, 0.5
    end.orientation.w = 1
    
    # 生成轨迹
    path, points = planner.plan_path(start, end, steps=20)
    # 输出路径点
    for i, pose in enumerate(path):
        print(f"Point {i}: [{pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}]")
    visualize_trajectory(np.array(points))
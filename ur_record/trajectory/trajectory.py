#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   trajectory.py
@Time    :   2025/07/23 15:33:46
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Quaternion
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import transforms3d as tf3d
import numpy as np

class TrajectoryPlanner:
    def __init__(self):
        pass

    def create_pose(self, xyz_: list, quat_wxyz_=None):
        """创建一个Pose对象"""
        pose_ = np.eye(4)
        pose_[:3, 3] = xyz_
        if quat_wxyz_ is None:
            quat_wxyz_ = [1, 0, 0, 0]
            pose_[:3, :3] = tf3d.quaternions.quat2mat(quat_wxyz_)  # wxyz顺序
        else:
            pose_[:3, :3] = tf3d.quaternions.quat2mat(quat_wxyz_)  # wxyz顺序
        return pose_
    
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

        return all_points, all_quats  # 返回路径点xyz, 四元数xyzw
    
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
    
    def visualize_trajectory(positions: np.array, orientations = None, step: int = 5, scale: float = 0.05):
        """
        可视化笛卡尔空间轨迹，支持方向标注
        参数:
        positions - Nx3数组, 轨迹点的XYZ坐标
        orientations - Nx4数组, 轨迹点的四元数(XYZW格式)，可选
        step - 方向箭头的绘制间隔（每隔几个点绘制一个）
        scale - 方向箭头的缩放比例
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        # 绘制轨迹
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], "bo-", linewidth=1, markersize=3, alpha=0.7, label="End-effector Path")

        # 添加起点和终点标记
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c="green", s=100, marker="o", edgecolors="k", label="Start")
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c="red", s=100, marker="*", edgecolors="k", label="End")
        # 添加方向标注（如果提供了四元数）
        if orientations is not None:
            # 确保四元数数量与位置点匹配
            if len(orientations) != len(positions):
                raise ValueError("位置点和方向四元数数量不匹配")

            # 每隔step个点绘制方向箭头
            for i in range(0, len(positions), step):
                pos = positions[i]
                quat_xyzw = orientations[i]

                # 创建旋转对象（使用XYZW格式）
                rotation = R.from_quat(quat_xyzw)

                # 获取坐标轴方向向量
                x_axis = rotation.apply([scale, 0, 0])
                y_axis = rotation.apply([0, scale, 0])
                z_axis = rotation.apply([0, 0, scale])

                # 绘制坐标轴
                ax.quiver(pos[0], pos[1], pos[2],
                        x_axis[0], x_axis[1], x_axis[2],
                        color="r", linewidth=1.5, alpha=0.8)
                ax.quiver(pos[0], pos[1], pos[2],
                        y_axis[0], y_axis[1], y_axis[2],
                        color="g", linewidth=1.5, alpha=0.8)
                ax.quiver(pos[0], pos[1], pos[2],
                        z_axis[0], z_axis[1], z_axis[2],
                        color="b", linewidth=1.5, alpha=0.8)
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
    # 示例用法
    planner = TrajectoryPlanner()

    start_pose = planner.create_pose([0.40593656521727806, 0.19103989555676446, 0.09998230819120782], [0.6022336272331124, -0.5738677670383182, -0.45149258095975353, 0.3227148796110952])
    end_pose = planner.create_pose([0.15371069, 0.19029899, -0.06846677], [0.6457788789185539, -0.5312803574078042, -0.37486812155046495, 0.40023082442580193])
    
    positions, orientations = planner.plan(start_pose, end_pose, steps=6, is_random=False, direction=[0.0, 0.0, 3.5])
    
    # 可视化轨迹
    positions_array = np.array(positions)
    TrajectoryPlanner.visualize_trajectory(positions_array)
    TrajectoryPlanner.visualize_trajectory(positions_array, orientations, step=1, scale=0.05)
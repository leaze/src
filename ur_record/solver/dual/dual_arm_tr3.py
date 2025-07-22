from scipy.spatial.transform import Rotation as R
from scipy.interpolate import make_interp_spline
from dual_arm_solver import ArmKinematics
from mpl_toolkits.mplot3d import Axes3D
from tracikpy import TracIKSolver
import matplotlib.pyplot as plt
import numpy as np


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

def slerp_orientations(q1, q2, steps):
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

def plan(start_pose, end_pose, steps=10, is_random=False, direction=[1.0, 1.0, 1.0]):
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
    orientations = slerp_orientations(start_quat, end_quat, steps)
    # 构建路径点
    all_points = []
    all_quats = []
    for i in range(steps):
        all_points.append(positions[i])
        # all_quats.append(orientations[i])

    return all_points

def visualize_trajectory(positions: np.array):
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
    # 创建左臂和右臂的运动学对象
    left_arm = ArmKinematics(is_left=True)
    right_arm = ArmKinematics(is_left=False)

    # 2. 逆向运动学：从位置和方向计算关节角度
    start_pos = [0.32746261711182717, 0.19675063469266912, -0.07188115117764517]
    start_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]
    end_pos = [0.2546261711182717, 0.15675063469266912, 0.07188115117764517]
    end_quat = [0.6549774920361782, -0.5350870364142088, -0.36644692369681464, 0.3878182570490953]

    # 生成轨迹
    all_points = []
    start_pose = np.eye(4)
    start_pose[:3, 3] = np.array(start_pos)
    start_pose[:3, :3] = quat_to_rot_matrix(start_quat)
    end_pose = np.eye(4)
    end_pose[:3, 3] = np.array(end_pos)
    end_pose[:3, :3] = quat_to_rot_matrix(end_quat)
    tr_points = plan(start_pose, end_pose, steps=10, is_random=False, direction=[1.0, 0.0, 1.0])
    for tr_point in tr_points:
        ik_joints = left_arm.inverse_kinematics(tr_point, start_quat)
        fk_xyz, _, _ = left_arm.forward_kinematics(ik_joints)
        diff = np.linalg.norm(fk_xyz - tr_point)
        print("diff = ", diff)
    visualize_trajectory(np.array(tr_points))
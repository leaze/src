from dual_arm_solver import ArmKinematics
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def interpolate_position(start_pos, end_pos, num_points):
    """线性插值位置"""
    return np.linspace(start_pos, end_pos, num_points)


def generate_trajectory_by_dist(start_pos_, end_pos_, dist_=0.02):
    """生成轨迹"""
    distance = np.linalg.norm(np.array(start_pos_) - np.array(end_pos_))
    num_points = int(distance / dist_)
    return interpolate_position(start_pos_, end_pos_, num_points)


def quaternion_slerp(q0, q1, t):
    """实现四元数的球面线性插值 (slerp)"""
    # 确保四元数是单位四元数
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    dot = np.dot(q0, q1)

    # 如果点积为负，反转其中一个四元数以取最短路径
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    # 防止数值误差
    dot = np.clip(dot, -1.0, 1.0)

    theta = np.arccos(dot)

    if theta < 1e-6:
        # 角度很小，线性插值足够
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    sin_theta = np.sin(theta)

    # 计算插值
    a = np.sin((1.0 - t) * theta) / sin_theta
    b = np.sin(t * theta) / sin_theta

    return a * q0 + b * q1


def interpolate_orientation(start_quat, end_quat, num_points):
    """手动实现四元数方向插值"""
    # 确保四元数是单位四元数
    start_quat = start_quat / np.linalg.norm(start_quat)
    end_quat = end_quat / np.linalg.norm(end_quat)

    times = np.linspace(0, 1, num_points)
    orientations = []

    for t in times:
        q = quaternion_slerp(start_quat, end_quat, t)
        orientations.append(q)

    return np.array(orientations)


def visualize_trajectory(positions):
    """可视化笛卡尔空间轨迹"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # 绘制轨迹
    ax.plot(
        positions[:, 0], positions[:, 1], positions[:, 2], "bo-", linewidth=1, markersize=3, alpha=0.7, label="End-effector Path"
    )

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


def check_ik_solution(arm, position, quaternion, joint_names):
    """检查IK解是否存在"""
    joints = arm.inverse_kinematics(position, quaternion)
    if joints is None:
        print(f"Error: No IK solution found for position {position} and orientation {quaternion}")
        return None

    # 检查关节限制
    limits = arm.joint_limits
    for i, angle in enumerate(joints):
        if angle < limits[i][0] or angle > limits[i][1]:
            print(
                f"Warning: Joint {joint_names[i]} ({np.rad2deg(angle):.2f}°) "
                f"outside limits [{np.rad2deg(limits[i][0]):.2f}°, {np.rad2deg(limits[i][1]):.2f}°]"
            )

    # 验证正向运动学
    pos, rot, quat = arm.forward_kinematics(joints)
    pos_error = np.linalg.norm(position - pos)
    if pos_error > 0.01:  # 1cm误差
        print(f"Warning: Position error {pos_error:.5f}m for IK solution")

    return joints


if __name__ == "__main__":
    # 创建运动学对象
    left_arm = ArmKinematics(is_left=True)

    # 假设的关节名称 - 请根据您的机械臂实际关节名修改
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    # 初始和终止位姿
    left_start_pos = np.array([5.04850000e-05, 2.64192055e-01, -1.77546768e-01])
    left_start_quat = np.array([9.99048232e-01, 4.36191563e-02, 0.00000000e00, -1.47352415e-16])
    left_end_pos = np.array([0.28298403, 0.24302717, 0.06437022])
    left_end_quat = np.array([0.706715, 0.03085568, -0.70615245, -0.03083112])

    # 检查起点和终点的IK解是否存在
    print("Checking start pose IK solution...")
    start_joints = check_ik_solution(left_arm, left_start_pos, left_start_quat, joint_names)

    print("\nChecking end pose IK solution...")
    end_joints = check_ik_solution(left_arm, left_end_pos, left_end_quat, joint_names)

    if start_joints is None or end_joints is None:
        print("\nError: Start or end pose is not feasible. Aborting trajectory generation.")
        exit(1)

    # 生成轨迹（2厘米生成一个点 - 减少数量以加快计算）
    positions = generate_trajectory_by_dist(left_start_pos, left_end_pos, 0.02)
    orientations = generate_trajectory_by_dist(left_start_quat, left_end_quat, 0.02)

    # 存储末端执行器位置用于可视化
    all_positions = []
    joint_trajectory = []

    # 跟踪前一步关节角度
    prev_joints = start_joints

    print("\nGenerating trajectory points...")
    for i, (pos, quat) in enumerate(zip(positions, orientations)):
        # print(f"\n--- Trajectory Point {i+1}/{num_points} ---")
        print(f"Position: {np.array_str(pos, precision=4, suppress_small=True)}")
        print(f"Quaternion: {np.array_str(quat, precision=4, suppress_small=True)}")

        # 使用上一步的解作为初始值
        joints = left_arm.inverse_kinematics(pos, quat, prev_joints)

        if joints is None:
            print("Warning: IK failed - using previous joints")
            joints = prev_joints
        else:
            prev_joints = joints

        joint_trajectory.append(joints)

        # 正向运动学验证
        calc_pos, _, _ = left_arm.forward_kinematics(joints)
        all_positions.append(calc_pos)

        print(f"FK Position: {np.array_str(calc_pos, precision=4, suppress_small=True)}")
        print("Joint Angles (deg):", np.rad2deg(joints).round(2))

    # 可视化笛卡尔空间轨迹
    visualize_trajectory(np.array(all_positions))

    print("\nTrajectory generation complete!")
    print(f"Saved visualization to trajectory_visualization.png")

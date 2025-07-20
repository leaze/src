from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np


def visualize_trajectory(positions: np.array):
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


if __name__ == "__main__":
    positions = np.array([[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [0.3, 0.3, 0.3]])
    visualize_trajectory(positions)
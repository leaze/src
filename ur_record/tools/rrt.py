import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import random


# ============================
# 环境设置
# ============================
class Environment:
    def __init__(self, start, goal, obstacles, bounds):
        """
        初始化环境参数
        start: 起点坐标 (x, y)
        goal: 终点坐标 (x, y)
        obstacles: 障碍物列表 [{'center':(x,y), 'radius':r}, ...]
        bounds: 环境边界 [x_min, x_max, y_min, y_max]
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.bounds = bounds


# ============================
# RRT算法实现
# ============================
class Node:
    def __init__(self, position, parent=None):
        self.position = np.array(position)  # 节点坐标
        self.parent = parent  # 父节点
        self.children = []  # 子节点
        self.cost = 0.0  # 从根节点到本节点的代价

    def __str__(self):
        return f"Node({self.position}, cost={self.cost:.2f})"


class RRTBase:
    def __init__(self, env, step_size=1.0, goal_sample_rate=0.05, max_iter=5000):
        self.env = env
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.nodes = []

    def reset(self):
        """重置算法状态"""
        self.nodes = []

    def plan(self):
        """主规划流程 (需在子类实现)"""
        raise NotImplementedError

    def generate_random_point(self):
        """生成随机采样点"""
        # 有一定概率直接采样目标点
        if random.random() < self.goal_sample_rate:
            return self.env.goal.copy()

        # 随机采样环境内的点
        x = random.uniform(self.env.bounds[0], self.env.bounds[1])
        y = random.uniform(self.env.bounds[2], self.env.bounds[3])
        return np.array([x, y])

    def find_nearest_node(self, point):
        """找到距离采样点最近的节点"""
        min_dist = float('inf')
        nearest_node = None

        for node in self.nodes:
            dist = np.linalg.norm(point - node.position)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node

        return nearest_node

    def steer(self, from_node, to_point):
        """从节点向目标点延伸一步"""
        direction = to_point - from_node.position
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            return to_point.copy()

        return from_node.position + self.step_size * direction / distance

    def is_collision_free(self, start, end):
        """检测两点之间的路径是否与障碍物碰撞"""
        # 分段检测路径上的点
        points = np.linspace(start, end, num=10)

        for point in points:
            for obstacle in self.env.obstacles:
                # 计算点到障碍物中心的距离
                dist_to_obs = np.linalg.norm(point - obstacle['center'])
                if dist_to_obs <= obstacle['radius']:
                    return False

        return True

    def plot(self, show_tree=True, path=None):
        """可视化算法结果"""
        plt.figure(figsize=(10, 10))

        # 绘制边界
        plt.xlim(self.env.bounds[0] - 1, self.env.bounds[1] + 1)
        plt.ylim(self.env.bounds[2] - 1, self.env.bounds[3] + 1)

        # 绘制起点和目标点
        plt.plot(*self.env.start, 'go', markersize=10, label='Start')
        plt.plot(*self.env.goal, 'ro', markersize=10, label='Goal')

        # 绘制障碍物
        for obstacle in self.env.obstacles:
            circle = Circle(obstacle['center'], obstacle['radius'],
                            color='gray', alpha=0.7)
            plt.gca().add_patch(circle)

        # 绘制树结构
        if show_tree:
            for node in self.nodes:
                if node.parent:
                    plt.plot([node.position[0], node.parent.position[0]],
                             [node.position[1], node.parent.position[1]],
                             'b-', linewidth=0.5, alpha=0.3)

        # 绘制最终路径
        if path is not None:
            plt.plot([p[0] for p in path], [p[1] for p in path],
                     'r-', linewidth=2, label='Path')

        plt.title(f"{self.__class__.__name__} Path Planning")
        plt.legend()
        plt.grid(True)
        plt.show()

    def trace_path(self, node):
        """从叶节点回溯到起点构建路径"""
        path = []
        while node is not None:
            path.append(node.position)
            node = node.parent
        return path[::-1]  # 反转路径


# ============================
# RRT算法
# ============================
class RRT(RRTBase):
    def plan(self):
        """RRT算法主循环"""
        self.reset()
        start_node = Node(self.env.start)
        self.nodes.append(start_node)

        for _ in range(self.max_iter):
            # 1. 随机采样点
            rand_point = self.generate_random_point()

            # 2. 寻找最近节点
            nearest_node = self.find_nearest_node(rand_point)

            # 3. 朝随机点方向延伸一步
            new_point = self.steer(nearest_node, rand_point)

            # 4. 碰撞检测
            if self.is_collision_free(nearest_node.position, new_point):
                # 5. 创建新节点
                new_node = Node(new_point, parent=nearest_node)
                new_node.cost = nearest_node.cost + np.linalg.norm(new_point - nearest_node.position)

                # 6. 添加到树中
                self.nodes.append(new_node)
                nearest_node.children.append(new_node)

                # 7. 检查是否到达目标区域
                dist_to_goal = np.linalg.norm(new_point - self.env.goal)
                if dist_to_goal <= self.step_size:
                    print(f"RRT: Path found in {_} iterations!")
                    return self.trace_path(new_node)

        print("RRT: Max iterations reached. Path not found.")
        return None


# ============================
# RRT*算法
# ============================
class RRTStar(RRTBase):
    def __init__(self, env, rewire_radius=5.0, **kwargs):
        super().__init__(env, **kwargs)
        self.rewire_radius = rewire_radius

    def plan(self):
        """RRT*算法主循环"""
        self.reset()
        start_node = Node(self.env.start)
        self.nodes.append(start_node)
        path = None

        for i in range(self.max_iter):
            # 1. 随机采样点
            rand_point = self.generate_random_point()

            # 2. 寻找最近节点
            nearest_node = self.find_nearest_node(rand_point)

            # 3. 朝随机点方向延伸一步
            new_point = self.steer(nearest_node, rand_point)

            # 4. 碰撞检测
            if not self.is_collision_free(nearest_node.position, new_point):
                continue

            # 5. 寻找近邻节点
            near_nodes = self.find_near_nodes(new_point)

            # 6. 选择最优父节点
            new_node = self.choose_parent(new_point, near_nodes, nearest_node)

            if new_node is None:
                continue

            # 7. 添加到树中
            self.nodes.append(new_node)

            # 8. 重布线优化
            self.rewire(new_node, near_nodes)

            # 9. 检查目标
            dist_to_goal = np.linalg.norm(new_node.position - self.env.goal)
            if dist_to_goal <= self.step_size:
                path = self.trace_path(new_node)
                print(f"RRT*: Path found in {i} iterations! Cost: {new_node.cost:.2f}")

        if path is None:
            print("RRT*: Max iterations reached. Path not found.")
        return path

    def find_near_nodes(self, point):
        """寻找新点附近的所有节点"""
        return [node for node in self.nodes
                if np.linalg.norm(point - node.position) <= self.rewire_radius]

    def choose_parent(self, new_point, near_nodes, nearest_node):
        """从附近节点中选择最优父节点"""
        min_cost = float('inf')
        best_parent = None

        # 检查每个附近的节点是否可以作为父节点
        for node in near_nodes:
            # 计算从该节点到新点的代价
            cost = node.cost + np.linalg.norm(new_point - node.position)

            # 碰撞检测
            if cost < min_cost and self.is_collision_free(node.position, new_point):
                min_cost = cost
                best_parent = node

        # 如果没有找到合适的节点，使用最近节点
        if best_parent is None:
            best_parent = nearest_node
            min_cost = best_parent.cost + np.linalg.norm(new_point - best_parent.position)

        # 创建新节点
        new_node = Node(new_point, best_parent)
        new_node.cost = min_cost

        # 添加到树中
        best_parent.children.append(new_node)
        return new_node

    def rewire(self, new_node, near_nodes):
        """重布线优化路径"""
        for node in near_nodes:
            # 如果节点是new_node本身或其父节点，跳过
            if node == new_node or node == new_node.parent:
                continue

            # 计算通过新节点的代价
            new_cost = new_node.cost + np.linalg.norm(node.position - new_node.position)

            # 如果新路径更优且无碰撞
            if new_cost < node.cost and self.is_collision_free(new_node.position, node.position):
                # 断开原父节点的连接
                if node.parent:
                    node.parent.children.remove(node)

                # 设置新的父节点关系
                node.parent = new_node
                node.cost = new_cost
                new_node.children.append(node)


# ============================
# 主程序
# ============================
if __name__ == "__main__":
    # 创建环境
    obstacles = [
        {'center': [15, 15], 'radius': 5},
        {'center': [35, 35], 'radius': 7},
        {'center': [60, 30], 'radius': 6},
        {'center': [40, 70], 'radius': 8},
        {'center': [70, 60], 'radius': 5}
    ]
    env = Environment(
        start=[10, 10],
        goal=[90, 90],
        obstacles=obstacles,
        bounds=[0, 100, 0, 100]
    )

    # 运行RRT算法
    rrt = RRT(env, step_size=3.0, max_iter=2000)
    rrt_path = rrt.plan()
    rrt.plot(show_tree=True, path=rrt_path)

    # 运行RRT*算法
    rrt_star = RRTStar(env, step_size=3.0, max_iter=2000, rewire_radius=15.0)
    rrt_star_path = rrt_star.plan()
    rrt_star.plot(show_tree=True, path=rrt_star_path)

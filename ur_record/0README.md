# 机器人双臂协同方案

```shell
# 这是我的机器人的urdf，我现在想操控机器人的双臂移动到指定位置，例如：左臂移动到(x1,y1,z1),右臂移动到(x2,y2,z3),然后这个时候会控制夹爪抓住货物后抬起来，然后机器人移动到制定为之后将货物放下来，请根据我的需求将任务进行分解，并且着重讲解一下我的双臂是如何进行协同作业的，我需要是ros2中实现
# src
# https://blog.csdn.net/qq_45762996/article/details/136472543
# https://2048.csdn.net/68219270a5baf817cf4a7b14.html?dp_token=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpZCI6MTI2NjAxNSwiZXhwIjoxNzUwMzA3MTU3LCJpYXQiOjE3NDk3MDIzNTcsInVzZXJuYW1lIjoiU3Rhcl9DaGVuZzA5MDEifQ.m7VaGqG021S55ksqPGPncHjzxrj9rebqxjwdb1rQIFE

# 2 tracikpy
在trac_ik_solver.txt的基础上继承TracIKSolver类，帮我实现类似于dual_arm_solver.txt中的ArmKinematics类，当逆解算失败时尝试使用不同的初始值，并且添加关节限位惩罚

class RobotIKSolver(TracIKSolver):
    def __init__(self, urdf_file, base_link, tip_link):
        super().__init__(urdf_file, base_link, tip_link)
```

## 1 任务分解

### 1.1 双臂协同规划​​

+ 左臂移动到抓取位置 (x1, y1, z1)
+ 右臂移动到抓取位置 (x2, y2, z2)
+ 确保双臂轨迹无碰撞且时间同步

### 1.2 ​抓取货物​​

+ 夹爪闭合控制（夹爪需在URDF中补充定义）
+ 抓取力度检测（压力传感器反馈）

### 1.3 ​抬升货物​​

+ 双臂协调提升货物（保持相对位姿不变）
+ 提升高度需考虑重心稳定

### 1.4 ​机器人导航​​

+ 移动底盘到目标位置（需补充移动底座URDF）
+ 使用Nav2实现SLAM导航

### 1.5 放置货物​​

+ 双臂协同下降到放置位置
+ 夹爪释放控制

## 2 双臂协同方案

```shell
# ROS2 包依赖
- MoveIt2：运动规划
- TF2：坐标系转换
- Control_msgs：关节控制
- ROS2 Control：硬件接口
```

```xml
<!-- ​补充夹爪模型​​（在wrist_roll_l_link和wrist_roll_r_link末端添加） -->
<!-- 示例：左夹爪 -->
<link name="gripper_left">
  <visual>...</visual>
  <collision>...</collision>
  <inertial>...</inertial>
</link>
<joint name="gripper_left_joint" type="prismatic">
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.1" effort="10"/>
</joint>
```

```xml
<!-- 添加移动底座​​（可选） -->
<link name="base_link"/>
<joint name="base_joint" type="fixed">
  <parent link="pelvis"/>
  <child link="base_link"/>
</joint>
```

## 3 协同作业实现流程

```shell
# 步骤1: 双臂运动规划​
# shell伪代码（使用MoveIt2 API）
from moveit_msgs.action import MoveGroup

# 左臂规划
left_arm_goal = MoveGroup.Goal()
left_arm_goal.request.target_pose.header.frame_id = "world"
left_arm_goal.request.target_pose.pose = create_pose(x1, y1, z1)

# 右臂规划
right_arm_goal = MoveGroup.Goal()
right_arm_goal.request.target_pose.header.frame_id = "world"
right_arm_goal.request.target_pose.pose = create_pose(x2, y2, z2)

# 同步执行
async with ActionClient(node, MoveGroup, 'left_arm/move_action') as left_client:
    async with ActionClient(node, MoveGroup, 'right_arm/move_action') as right_client:
        await left_client.send_goal_async(left_arm_goal)
        await right_client.send_goal_async(right_arm_goal)
        await asyncio.gather(
            left_client.get_result(),
            right_client.get_result()
        )
```

```shell
# 步骤2: 抓取控制​
# 夹爪控制服务（自定义Action）
action = GripperCommand.Goal()
action.command.position = 0.02  # 闭合距离
left_gripper_client.send_goal(action)
right_gripper_client.send_goal(action)

# 抓取检测（需压力传感器）
while pressure_sensor.value < threshold:
    rclpy.spin_once(node)  # 等待抓取完成
```

```shell
# 步骤3: 抬升货物​
# 保持相对位姿抬升
current_pose = get_ee_poses()  # 获取当前末端位姿

# 计算抬升轨迹（Z轴+0.2m）
lift_traj = [
    interpolate_pose(current_pose.left, z_offset=0.2),
    interpolate_pose(current_pose.right, z_offset=0.2)
]

# 执行协同运动
execute_trajectory_sync(lift_traj)
```

```shell
# 步骤4: 导航移动​
# 使用Nav2导航
ros2 launch nav2_bringup navigation_launch.py
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: x3, y: y3, z: 0}}}"
```

```shell
# 步骤5: 放置货物
# 下降到放置高度
place_traj = [
    create_pose(x3, y3, z_place + 0.1),  # 放置点上方
    create_pose(x3, y3, z_place)          # 最终位置
]

# 夹爪释放
gripper_goal.command.position = 0.1  # 张开距离
```

## 4 协同作业关键技术

```shell
# 1 双臂协调策略​
# 相对约束控制​​：保持夹爪间相对位姿不变
constraint = moveit_msgs.msg.Constraints()
constraint.position_constraints.append(
  create_position_constraint("gripper_left", "gripper_right")
)
move_group.set_path_constraints(constraint)
# 使用时间最优轨迹规划
move_group.set_planner_id("RRTConnectkConfigDefault")
move_group.set_max_velocity_scaling_factor(0.5)
# 2 碰撞避免
# 添加碰撞物体
collision_object = moveit_msgs.msg.CollisionObject()
collision_object.id = "cargo"
collision_object.header.frame_id = "world"
collision_object.primitives.append(create_box(0.5, 0.3, 0.2))
planning_scene.add_object(collision_object)
# ​​3. 容错机制​​
# 抓取失败检测（压力/视觉反馈）
# 轨迹重规划（当碰撞发生时）
# 阻抗控制（接触力感知）
```

## 5 系统架构建议

```shell
                     +----------------+
                     |   Task Planner |  (Behavior Tree)
                     +--------+-------+
                              |
                     +--------v-------+
                     |  Dual-Arm     |  (MoveIt2 Server)
                     |  Coordinator  |
                     +--------+-------+
                              |
        +---------------------+---------------------+
        |                     |                     |
+-------v-------+    +--------v--------+    +-------v-------+
| Left Arm      |    | Navigation      |    | Right Arm     |
| Controller    |    | (Nav2)          |    | Controller   |
+-------+-------+    +--------+--------+    +-------+-------+
        |                     |                     |
+-------v-------+    +--------v--------+    +-------v-------+
| ROS2 Control  |    | Mobile Base     |    | ROS2 Control  |
| (Left Arm)    |    | Controller      |    | (Right Arm)   |
+---------------+    +-----------------+    +---------------+
```

## 6 IK实现思路

<https://bbs.realman-robotics.cn/question/326.html>

### arm_solver逆解算

![ik_solver](../data/img/ik_solver.png)

### arm_solver_que逆解算

![ik_solver_que](../data/img/ik_solver_que.png)
![my_solver2](../data/img/my_solver2.png)

## 7 任务计划

+ [x] 1 两只机械手抓取物料
  + [x] 1.1 控制双臂移动到指定位置: 阅读dual_arm_demo源码
  + [x] 1.2 获取末端执行其的位置、对应各个关节角的角度
  + [x] 1.2.3 控制夹爪抓住楔子: 灵巧手接口
  + [x] 1.2.4 控制双臂向前推进: 控制器同比步运动
  + [x] 1.2.5 控制夹爪松开楔子
  + [x] 1.2.6 控制双臂移动到指定位置
  + [x] 1.2.7 控制夹爪抓住纸箱
  + [x] 1.2.8 控制双臂抬起纸箱
  + [x] 1.2.9 控制双臂向前后左右上下移动固定距离
+ [ ] 2 将物料放到制定位置
  + [ ] 2.1 通过视觉模块获取目标位置
  + [x] 2.2 获取末端执行器运动的位置
  + [x] 2.3 控制双臂同步运动到指定位置
  + [ ] 2.4 控制夹爪移动到纸箱顶点
  + [x] 2.5 控制双臂抬起纸箱
+ [x] 3 调度
  + [x] 3.1 导航到指定位置 nav_rosbridge类的接口
+ [ ] 4 正逆解算优化
  + [ ] 4.1 避障 <https://yuanbao.tencent.com/chat/naQivTmsDa/1a29c449-7aec-4e23-92cc-ce8b33a20d01>
  + [x] 4.2 奇异值优化 <https://chat.deepseek.com/a/chat/s/5d8960b1-2550-4fb1-b334-e9b5f464230c>
  + [x] 4.3 双臂协同轨迹优化 <https://yuanbao.tencent.com/chat/naQivTmsDa/faaa8bd6-973b-4648-a58a-5d403b7bbe15>
  + [ ] 4.4 PID / LQR 位控
  + [ ] 4.5 力控
+ [ ] 5 ROS2 控制
  + [x] 5.1 重新反工ROS1代码
  + [ ] 5.2 上料任务
+ [ ] 6 联调
  + [ ] 6.1 获取视觉反馈

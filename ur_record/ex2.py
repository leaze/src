import mujoco
import mujoco.viewer
import numpy as np
import time

# 加载模型
model = mujoco.MjModel.from_xml_path('./description/tiangong_description/xml/arm.xml')
data = mujoco.MjData(model)

# 获取所有执行器的名称列表
actuator_names = [model.actuator(i).name for i in range(model.nu)]

# 创建关节名称到执行器ID的映射
joint_to_actuator = {}
for i, name in enumerate(actuator_names):
    joint_name = name.replace("_actuator", "")  # 根据实际情况调整
    joint_to_actuator[joint_name] = i

# 定义双臂控制函数
def set_arm_positions(left_arm_positions, right_arm_positions):
    """设置双臂关节位置"""
    # 左臂关节名称列表
    left_joint_names = [
        "shoulder_pitch_l_joint",
        "shoulder_roll_l_joint",
        "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint",
        "elbow_yaw_l_joint",
        "wrist_pitch_l_joint",
        "wrist_roll_l_joint"
    ]
    
    # 右臂关节名称列表
    right_joint_names = [
        "shoulder_pitch_r_joint",
        "shoulder_roll_r_joint",
        "shoulder_yaw_r_joint",
        "elbow_pitch_r_joint",
        "elbow_yaw_r_joint",
        "wrist_pitch_r_joint",
        "wrist_roll_r_joint"
    ]
    
    # 设置左臂关节位置
    for i, name in enumerate(left_joint_names):
        joint_id = model.joint(name).id
        data.qpos[joint_id] = left_arm_positions[i]
    
    # 设置右臂关节位置
    for i, name in enumerate(right_joint_names):
        joint_id = model.joint(name).id
        data.qpos[joint_id] = right_arm_positions[i]

# 定义双臂运动序列
arm_positions = [
    # 初始姿势
    ([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]),
    
    # 挥手姿势
    ([0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0], [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]),
    
    # 抓取姿势
    ([0.0, 0.3, 0.5, -1.2, 0.5, -0.5, 0.3], [0.0, -0.3, -0.5, -1.2, -0.5, 0.5, -0.3]),
    
    # T型姿势
    ([0.0, 1.57, 0.0, -1.57, 0.0, 0.0, 0.0], [0.0, -1.57, 0.0, -1.57, 0.0, 0.0, 0.0]),
]

# 创建查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 设置相机位置
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20
    
    # 稳定机器人（先运行100步）
    for _ in range(100):
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # 运动循环
    for left_pos, right_pos in arm_positions:
        # 设置目标位置
        set_arm_positions(left_pos, right_pos)
        
        # 使用PD控制器平滑移动到目标位置
        for _ in range(300):  # 300步约为0.3秒
            # 计算控制信号（简单PD控制器）
            kp = 100.0  # 比例增益
            kd = 10.0   # 微分增益
            
            # 更新左臂控制
            for i, name in enumerate([
                "shoulder_pitch_l_joint",
                "shoulder_roll_l_joint",
                "shoulder_yaw_l_joint",
                "elbow_pitch_l_joint",
                "elbow_yaw_l_joint",
                "wrist_pitch_l_joint",
                "wrist_roll_l_joint"
            ]):
                # 获取关节ID
                joint_id = model.joint(name).id
                
                # 获取执行器ID
                actuator_id = joint_to_actuator.get(name, -1)
                if actuator_id == -1:
                    continue
                
                # 位置误差
                pos_error = data.qpos[joint_id] - left_pos[i]
                
                # 速度误差
                vel_error = data.qvel[joint_id]
                
                # 计算控制力
                data.ctrl[actuator_id] = -kp * pos_error - kd * vel_error
            
            # 更新右臂控制
            for i, name in enumerate([
                "shoulder_pitch_r_joint",
                "shoulder_roll_r_joint",
                "shoulder_yaw_r_joint",
                "elbow_pitch_r_joint",
                "elbow_yaw_r_joint",
                "wrist_pitch_r_joint",
                "wrist_roll_r_joint"
            ]):
                # 获取关节ID
                joint_id = model.joint(name).id
                
                # 获取执行器ID
                actuator_id = joint_to_actuator.get(name, -1)
                if actuator_id == -1:
                    continue
                
                # 位置误差
                pos_error = data.qpos[joint_id] - right_pos[i]
                
                # 速度误差
                vel_error = data.qvel[joint_id]
                
                # 计算控制力
                data.ctrl[actuator_id] = -kp * pos_error - kd * vel_error
            
            # 步进模拟
            mujoco.mj_step(model, data)
            
            # 更新查看器
            viewer.sync()
            time.sleep(0.001)  # 控制模拟速度
        
        # 保持姿势1秒
        for _ in range(1000):
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)

    # 保持窗口打开
    print("运动序列完成，按ESC退出...")
    while viewer.is_running():
        time.sleep(0.1)
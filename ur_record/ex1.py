import mujoco
import mujoco.viewer
import numpy as np
import time

# 加载模型
model = mujoco.MjModel.from_xml_path('./description/tiangong_description/xml/arm.xml')
data = mujoco.MjData(model)
data.qpos[model.joint("waist_yaw_joint").id] = (0.0)
# 定义双臂控制函数
def set_arm_positions(left_arm_positions, right_arm_positions):
    """设置双臂关节位置"""
    # 左臂关节索引
    left_joints = [
        model.joint("shoulder_pitch_l_joint").id,
        model.joint("shoulder_roll_l_joint").id,
        model.joint("shoulder_yaw_l_joint").id,
        model.joint("elbow_pitch_l_joint").id,
        model.joint("elbow_yaw_l_joint").id,
        model.joint("wrist_pitch_l_joint").id,
        model.joint("wrist_roll_l_joint").id
    ]
    
    # 右臂关节索引
    right_joints = [
        model.joint("shoulder_pitch_r_joint").id,
        model.joint("shoulder_roll_r_joint").id,
        model.joint("shoulder_yaw_r_joint").id,
        model.joint("elbow_pitch_r_joint").id,
        model.joint("elbow_yaw_r_joint").id,
        model.joint("wrist_pitch_r_joint").id,
        model.joint("wrist_roll_r_joint").id
    ]
    
    # 设置关节位置
    data.qpos[left_joints] = left_arm_positions
    data.qpos[right_joints] = right_arm_positions

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
    data.qpos[model.joint("waist_yaw_joint").id] = (0.0)
    # 稳定机器人（先运行100步）
    for _ in range(100):
        mujoco.mj_step(model, data)
        viewer.sync()
    
    # 运动循环
    for left_pos, right_pos in arm_positions:
        # 设置目标位置
        set_arm_positions(left_pos, right_pos)
        data.qpos[model.joint("waist_yaw_joint").id] = (0.0)
        # 使用PD控制器平滑移动到目标位置
        for _ in range(300):  # 300步约为0.3秒
            # 计算控制信号（简单PD控制器）
            for i in range(model.nu):
                joint_id = model.actuator_trnid[i, 0]
                kp = 100.0  # 比例增益
                kd = 10.0   # 微分增益
                
                # 位置误差
                pos_error = data.qpos[joint_id]
                
                # 速度误差
                vel_error = data.qvel[joint_id] 
                
                # 计算控制力
                data.ctrl[i] = -kp * pos_error - kd * vel_error
            
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
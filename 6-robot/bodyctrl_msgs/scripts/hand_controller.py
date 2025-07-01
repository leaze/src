#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   hand_controller.py
@Time    :   2025/07/01 13:28:45
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class InspireHand:
    """灵巧手控制类"""
    def __init__(self, hand_side='left'):
        """
        初始化灵巧手控制器
        :param hand_side: 'left' 或 'right'，指定是左手还是右手
        """
        self.hand_side = hand_side
        self.joint_names = [
            "little_finger",  # 小指 (0)
            "ring_finger",    # 无名指 (1)
            "middle_finger",  # 中指 (2)
            "fore_finger",    # 食指 (3)
            "thumb_bend",     # 拇指弯曲 (4)
            "thumb_rotation"  # 拇指旋转 (5)
        ]
        
        # 关节状态 (位置, 速度, 电流) 百分比值 [0-100]
        self.positions = np.zeros(6, dtype=float)  # 位置百分比
        self.velocities = np.zeros(6, dtype=float) # 速度百分比
        self.efforts = np.zeros(6, dtype=float)    # 电流百分比
        
        # 目标位置 (用于位置控制)
        self.target_positions = np.zeros(6, dtype=float)
        
        # ROS设置
        rospy.init_node(f'inspire_hand_{hand_side}', anonymous=True)
        
        # 状态发布器
        self.state_pub = rospy.Publisher(
            f'/inspire_hand/ctrl/{hand_side}_hand', 
            JointState, 
            queue_size=10
        )
        
        # # 命令订阅器
        rospy.Subscriber(
            f'/inspire_hand/ctrl/{hand_side}_hand', 
            JointState, 
            self.hand_info_callback
        )
        rospy.Subscriber(
            f'/inspire_hand/cmd_gesture', 
            String, 
            self.gesture_cmd_callback
        )
        
        rospy.loginfo(f"{hand_side.capitalize()} hand controller initialized")
    
    def update_state(self):
        """更新手指状态 - 模拟从硬件读取数据"""
        # 模拟真实的手指行为
        for i in range(6):
            # 位置逐渐趋向目标值
            if abs(self.positions[i] - self.target_positions[i]) > 1.0:
                # 计算移动方向
                direction = 1 if self.target_positions[i] > self.positions[i] else -1
                
                # 更新位置 (最大移动5%/周期)
                move_step = min(5.0, abs(self.target_positions[i] - self.positions[i]))
                self.positions[i] += direction * move_step
                
                # 速度随位置差异变化
                self.velocities[i] = move_step * 2  # 简单的速度模拟
            else:
                self.velocities[i] = 0.0
                
            # 电流模拟：与速度和位置有关
            # 在接近目标位置时电流减小
            distance_factor = abs(self.target_positions[i] - self.positions[i]) / 1.0
            # 在高速移动时电流增大
            speed_factor = min(1.0, self.velocities[i] / 20.0)
            # 基础电流 + 速度相关电流 + 位置相关电流
            self.efforts[i] = 10.0 + 30.0 * speed_factor + 10.0 * distance_factor
            
            # 确保百分比在0-100之间
            self.positions[i] = max(0.0, min(1.0, self.positions[i]))
            self.velocities[i] = max(0.0, min(1.0, self.velocities[i]))
            self.efforts[i] = max(0.0, min(1.0, self.efforts[i]))
    
    def publish_state(self):
        """发布手指状态到ROS话题"""
        state_msg = JointState()
        
        # 设置消息头
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = f""
        
        # 设置关节名称
        state_msg.name = self.joint_names
        
        # 设置位置、速度和电流百分比值
        state_msg.position = list(self.positions)
        state_msg.velocity = list(self.velocities)
        state_msg.effort = list(self.efforts)
        
        # 发布消息
        self.state_pub.publish(state_msg)
    
    def hand_info_callback(self, msg:JointState):
        """位置命令回调函数"""
        # if len(msg) != 6:
        #     rospy.logwarn(f"Invalid position command size: {len(msg)} instead of 6")
        #     return
        
        self.target_positions = np.array(msg.position)
        self.velocity = np.array(msg.position)
        self.effort = np.array(msg.position)
        rospy.loginfo(f"Position command received: {self.target_positions}")
        rospy.loginfo(f"Velocity command received: {self.velocity}")
        rospy.loginfo(f"Force command received: {self.effort}")
    
    
    
    def gesture_cmd_callback(self, msg:String):
        """手势命令回调函数"""
        gesture = msg.data.lower()
        rospy.loginfo(f"Gesture command received: {gesture}")
        
        # 预定义手势
        if gesture == 'fist':
            # 拳头手势 - 所有手指闭合
            self.target_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5], dtype=float)
        elif gesture == 'open_hand':
            # 五指张开
            self.target_positions = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.0], dtype=float)
        elif gesture == 'ok_sign':
            # OK手势
            self.target_positions = np.array([1.0, 1.0, 1.0, 0.0, 1.0, 0.0], dtype=float)
        elif gesture == 'scissors':
            # 剪刀手势
            self.target_positions = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0], dtype=float)
        elif gesture == 'thumbs_up':
            # 点赞手势
            self.target_positions = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 1.0], dtype=float)
        else:
            rospy.logwarn(f"Unknown gesture: {gesture}")

def hand_control_node(hand_side):
    """运行灵巧手控制节点"""
    hand = InspireHand(hand_side)
    rate = rospy.Rate(10)  # 10Hz
    
    rospy.loginfo(f"Starting {hand_side} hand control node...")
    
    while not rospy.is_shutdown():
        # 更新手部状态
        # hand.update_state()
        
        # 发布状态
        # hand.publish_state()
        
        # 休眠以保持频率
        rate.sleep()

if __name__ == "__main__":
    import sys
    
    # 从命令行参数获取手部方向
    hand_side = 'left'
    # if len(sys.argv) > 1 and sys.argv[1] in ['left', 'right']:
    #     hand_side = sys.argv[1]
    
    try:
        hand_control_node(hand_side)
    except rospy.ROSInterruptException:
        pass
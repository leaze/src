#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np

joint_names = [
    "little_finger",  # 小指 (0)
    "ring_finger",    # 无名指 (1)
    "middle_finger",  # 中指 (2)
    "fore_finger",    # 食指 (3)
    "thumb_bend",     # 拇指弯曲 (4)
    "thumb_rotation"  # 拇指旋转 (5)
]
def send_position_command(hand_side, positions, velocities, efforts):
    """发送位置命令到指定手部"""
    state_pub = rospy.Publisher(f'/inspire_hand/ctrl/{hand_side}_hand', JointState, queue_size=10)
    # 等待连接
    rospy.sleep(0.5)
    rospy.loginfo(f"Position command sent to {hand_side} hand: {positions} {velocities} {efforts}")
    """发布手指状态到ROS话题"""
    state_msg = JointState()
    
    # 设置消息头
    state_msg.header.stamp = rospy.Time.now()
    state_msg.header.frame_id = f""
    
    # 设置关节名称
    state_msg.name = joint_names
    state_msg.position = list(positions)
    state_msg.velocity = list(velocities)
    state_msg.effort = list(efforts)
    
    # 发布消息
    state_pub.publish(state_msg)

def test_hand_commands():
    """测试各种手部命令"""
    rospy.init_node('hand_tester', anonymous=True)
    # 设置位置、速度和电流百分比值
    positions = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]  # 位置百分比
    velocities = [0.2, 0.2, 0.2, 0.2, 0.2, 0.0] # 速度百分比
    efforts = [0.2, 0.2, 0.2, 0.2, 0.2, 0.0]    # 电流百分比
    # 测试左手命令
    send_position_command('left', [1.0, 1.0, 1.0, 1.0, 1.0, 0.0], velocities, efforts)  # 张开左手
    rospy.sleep(2.0)
    send_position_command('left', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocities, efforts)  # 闭合左手
    
    # 测试右手命令
    send_position_command('right', [0.0, 0.0, 1.0, 1.0, 0.0, 0.0], velocities, efforts)  # 右手剪刀手
    rospy.sleep(2.0)
    send_position_command('right', [1.0, 1.0, 1.0, 0.0, 1.0, 0.0], velocities, efforts)  # 右手OK手势

if __name__ == '__main__':
    try:
        test_hand_commands()
    except rospy.ROSInterruptException:
        pass
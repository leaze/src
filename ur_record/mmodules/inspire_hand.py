#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   inspire_hand.py
@Time    :   2025/06/28 19:21:39
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from bodyctrl_msgs.srv import set_angle, set_angleRequest, set_angleResponse
from sensor_msgs.msg import JointState
import rospy

class InspireHandController:
    def __init__(self, is_left=True):
        """
        Inspire Hand ROS控制器
        :param hand: 'left' 或 'right'，指定控制的手
        """
        self.hand = 'left' if is_left else 'right'
        self.current_state = None
        self.name = None
        self.position = None
        self.velocity = None
        self.effort = None
        
        # 状态订阅者
        self.state_sub = rospy.Subscriber(
            f"/inspire_hand/state/{self.hand}_hand",
            JointState,
            self.state_callback
        )
        
        # 初始化服务代理（等待服务就绪）
        # rospy.wait_for_service(f"{self.hand}/set_angle")

        # 角度设置服务代理
        self.set_angle_service = rospy.ServiceProxy(f"/inspire_hand/set_angle/{self.hand}",set_angle)
        rospy.sleep(0.1)
    def state_callback(self, msg:JointState):
        """处理关节状态更新"""
        self.current_state = msg
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
    def get_hand_state(self) -> tuple:
        """ 获取手的当前状态
        :return: (name, position, velocity, effort) 分别对应关节名称、位置、速度和力矩
        """
        if self.current_state is None:
            rospy.logwarn("Hand No state data received yet")
        return self.name, self.position, self.velocity, self.effort

    def get_finger_state(self, finger_idx) -> tuple:
        """
        获取手指的当前状态
        :param finger_idx: 手指索引 (0-5: 0小指、1-无名指、2-中指、3-食指、4-拇指弯曲、5-拇指旋转)
        :return: (position, velocity, effort) 单位均为百分比
        """
        if self.current_state is None:
            rospy.logwarn("Finger No state data received yet")
            return None, None, None
        return self.position[finger_idx], self.velocity[finger_idx], self.effort[finger_idx]
            

    def set_angles(self, angles:list) -> bool:
        """
        设置手指角度
        :param angles: 长度为6的列表, 对应各手指角度百分比[0.0-1.0]
                       [小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转]
        :return: 是否设置成功
        """
        try:
            req = set_angleRequest()
            # 假设角度设置服务的参数结构与力矩设置类似
            if len(angles) != 6:
                rospy.logerr("Angles list must contain exactly 6 values")
                return False
            req.angle0Ratio = angles[0]  # 小指
            req.angle1Ratio = angles[1]  # 无名指
            req.angle2Ratio = angles[2]  # 中指
            req.angle3Ratio = angles[3]  # 食指
            req.angle4Ratio = angles[4]  # 拇指弯曲
            req.angle5Ratio = angles[5]  # 拇指旋转
            
            resp = self.set_angle_service(req)
            rospy.loginfo(f"Angle set{'suceess' if resp.angle_accepted else 'failed'}")
            return resp.angle_accepted
        except rospy.ServiceException as e:
            rospy.logerr(f"Angle set service call failed: {e}")
            return False

# 使用示例
if __name__ == "__main__":
    rospy.init_node('inspire_hand_controller')
    # 创建右手控制器
    right_hand = InspireHandController(is_left=False)
    # 获取五指状态
    name_ls, pos_ls,speed_ls,effort_ls = right_hand.get_hand_state()
    # 获取手指状态
    pos, speed, effort = right_hand.get_finger_state(0)  # 获取小指状态
    # 设置手指角度
    angles = [0.8, 0.8, 0.9, 0.8, 0.7, 0.5]  # 80%-90%打开
    success = right_hand.set_angles(angles)
    rospy.loginfo(f"Set angles {'succeeded' if success else 'failed'}")
    rospy.spin()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   motion_mode.py
@Time    :   2025/07/15 13:08:35
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
from hric_msgs.srv import SetMotionMode, SetMotionModeRequest
from hric_msgs.msg import MotionStatus
import rospy
import time


class MotionMode:
    def __init__(self):
        # 等待运动模式服务可用
        rospy.wait_for_service('/hric/motion/set_motion_mode')
        self.set_mode_service = rospy.ServiceProxy('/hric/motion/set_motion_mode', SetMotionMode)
        
        # 运动状态
        self.motion_status = None
        self.walk_mode = 0  # 行走状态: 0-start, 1-stop, 2-zero, 3-zero_2_stand, 4-stand, 5-stand_2_walk, 6-walk
        self.is_console_control = True
        self.is_swing_arm = True
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # 设置模式
        self.timeout = rospy.get_param("~timeout", 5.0)

        # 状态订阅
        self.motion_status_sub = rospy.Subscriber('/hric/motion/status', MotionStatus, self.motion_status_callback)
        rospy.sleep(0.1)
        
    def motion_status_callback(self, msg: MotionStatus):
        self.walk_mode = msg.walk_mode
        self.motion_status = msg
        self.is_console_control = msg.is_console_control
        self.is_swing_arm = msg.is_swing_arm
        self.linear_x = msg.velocity.linear.x
        self.linear_y = msg.velocity.linear.y
        self.angular_z = msg.velocity.angular.z

    def set_motion_mode(self, mode_request, swing_arm=False):
        """
        :param mode_request: 运动模式请求
        0: start
        1: stop
        2: zero
        3: stand
        4: walk
        """
        req = SetMotionModeRequest()
        req.walk_mode_request = mode_request
        req.is_need_swing_arm = swing_arm
        
        try:
            response = self.set_mode_service(req)
            if response.success:
                rospy.loginfo(f"Mode {mode_request} set successfully!")
            else:
                rospy.logerr(f"Failed to set mode {mode_request}, error code: {response.error_code}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")
            return False
    
    def set_stop_mode(self):
        """发送停止模式设置请求"""
        success = self.set_motion_mode(1)
        start_time = time.time()
        timeout = self.timeout
        while time.time() - start_time < timeout:
            if success and self.walk_mode == 1:
                rospy.loginfo("Stop mode set successfully")
                return success
        rospy.logwarn("Timeout while setting stop mode")
        return False  # 超时未成功，返回False
    def set_zero_mode(self):
        """发送回零模式设置请求"""
        success = self.set_motion_mode(2)
        start_time = time.time()
        timeout = self.timeout
        while time.time() - start_time < timeout:
            if success and self.walk_mode == 2:
                rospy.loginfo("Zero mode set successfully")
                return success
        rospy.logwarn("Timeout while setting zero mode")
        return False  # 超时未成功，返回False

    def set_stand_mode(self):
        """发送站立模式设置请求"""
        success = self.set_motion_mode(3)
        start_time = time.time()
        timeout = self.timeout
        while time.time() - start_time < timeout:
            if success and self.walk_mode == 4:
                rospy.loginfo("Stand mode set successfully")
                return success
            rospy.sleep(0.1)  # 小间隔避免占用过多资源
        rospy.logwarn("Timeout while setting stand mode")
        return False  # 超时未成功，返回False

    def set_walk_mode(self):
        """发送行走模式设置请求"""
        success = self.set_motion_mode(4)
        start_time = time.time()
        timeout = self.timeout
        while time.time() - start_time < timeout:
            if success and self.walk_mode == 6:
                rospy.loginfo("Walk mode set successfully")
                return success
        rospy.logwarn("Timeout while setting walk mode")
        return False


if __name__ == "__main__":
    rospy.init_node('motion_mode')
    controller = MotionMode()
    rospy.spin()
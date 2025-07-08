#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""
@File    :   main.py
@Time    :   2025/06/28 19:43:09
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
"""
from controllers.arm_controller import ArmController
from mmodules.inspire_controller import InspireController
from sensors.arm_6dof import Arm6Dof
import rospy


class RobotController:
    def __init__(self):
        self.arm_controller = ArmController()
        self.hand_controller = InspireController()
        self.arm_6dof = Arm6Dof()
        self.current_state = None
        rospy.sleep(0.1)

    def recveive_command(self):
        # 接收指令
        pass

    def location_wedge(self):
        # 定位楔子
        left_target_pos_ = [0.28298422, 0.24302539, 0.06438034]
        left_target_quat_ = [0.7065502655329249, 0.031041074367685273, -0.7063179757461218, -0.03062827313698852]
        right_target_pos_ = [0.28297577, -0.18721895, 0.05214833]
        right_target_quat_ = [0.7068686341423475, 0.03111171108433466, -0.7059985709335345, -0.030574717315850895]
        rospy.loginfo("Location Wedge suceessed")
        success_left, success_right = self.arm_controller.move_dual_arm(left_target_pos_, left_target_quat_, right_target_pos_, right_target_quat_)
        rospy.loginfo(f"Move dual arm succeeded: {success_left} {success_right}") if success_left and success_right else rospy.logerr(f"Move dual arm failed: {success_left} {success_right}")
        return success_left and success_right

    def grab_wedge(self):
        # 旋转腕部
        self.arm_controller.rotate_single_joint(True, 6, rotate_angle=0.0)  # 左腕旋转
        self.arm_controller.rotate_single_joint(False, 6, rotate_angle=-0.0)  # 右腕旋转
        # 抓取楔子
        grab_left_wedge_status, grab_right_wedge_status = self.hand_controller.move_both_hands([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rospy.loginfo("Grab wedge succeeded") if grab_left_wedge_status and grab_right_wedge_status else rospy.logerr("Grab wedge failed")
        # 抬起楔子
        move_up_wedge_status = self.arm_controller.move_up(0.05)
        rospy.loginfo("Move up wedge succeeded") if move_up_wedge_status else rospy.logerr("Move up wedge failed")
        return grab_left_wedge_status and grab_right_wedge_status and move_up_wedge_status

    def move_to_box(self):
        # 移动到纸箱前
        move_to_box_backward_status = self.arm_controller.move_backward(0.05)
        rospy.loginfo("Move backward wedge succeeded") if move_to_box_backward_status else rospy.logerr("Move backward wedge failed")
        move_to_box_down_status = self.arm_controller.move_down(0.05)
        rospy.loginfo("Move down wedge succeeded") if move_to_box_down_status else rospy.logerr("Move down wedge failed")
        return move_to_box_backward_status and move_to_box_down_status

    def insert_wedge(self):
        # 插入楔子
        inser_wedge_status = self.arm_controller.move_forward(0.05)
        rospy.loginfo("Move forward wedge succeeded") if inser_wedge_status else rospy.logerr("Move forward wedge failed")
        return inser_wedge_status

    def grab_box(self):
        # 抓取纸箱
        release_wedge_status = self.hand_controller.release_both_hands()
        rospy.loginfo("Release wedge succeeded") if release_wedge_status else rospy.logerr("Release wedge failed")
        move_left_status = self.arm_controller.move_single_left(is_left=True, distance=0.01)  # 左臂左移
        move_right_status = self.arm_controller.move_single_right(is_left=False, distance=0.01)  # 右臂右移
        move_up_status = self.arm_controller.move_up(0.01)
        # 旋转腕部
        self.arm_controller.rotate_single_joint(True, 6, rotate_angle=0.0)  # 左腕旋转
        self.arm_controller.rotate_single_joint(False, 6, rotate_angle=-0.0)  # 右腕旋转
        grab_box_status = self.hand_controller.grip_both_hands()
        rospy.loginfo("Grab box succeeded") if grab_box_status else rospy.logerr("Grab box failed")
        return release_wedge_status and move_left_status and move_right_status and move_up_status and grab_box_status

    def move_box(self):
        # 移动纸箱的逻辑
        move_box_status = self.arm_controller.move_down(0.01)
        return move_box_status

    def place_box(self):
        # 置纸箱的逻辑
        pass

    def lift_box(self, distance):
        # 向上移动的逻辑
        return self.arm_controller.move_up(distance)

    def test(self):
        # 移动各个关节轴
        all_positions = [0.0] * 14  # 初始化所有关节位置为0
        all_positions[3] = -1.57
        all_positions[10] = -1.57
        self.arm_controller.send_arms_cmd_pos(self.arm_controller.joint_names[True] + self.arm_controller.joint_names[False], all_positions, [self.arm_controller.joint_speed] * 14, [self.arm_controller.joint_current] * 14)
        rospy.sleep(2.0)
        # 旋转腕部
        self.arm_controller.rotate_single_joint(True, 4, rotate_angle=1.57)  # 左腕旋转
        self.arm_controller.rotate_single_joint(False, 4, rotate_angle=-1.57)  # 右腕旋转
        self.arm_controller.rotate_single_joint(True, 2, rotate_angle=-0.2)  # 左腕旋转
        self.arm_controller.rotate_single_joint(False, 2, rotate_angle=0.2)  # 左腕旋转
        # 灵巧手
        self.hand_controller.move_both_hands([0.1, 0.1, 0.1, 0.1, 0.5, 0.5], [0.1, 0.1, 0.1, 0.1, 0.5, 0.5])


if __name__ == "__main__":
    rospy.init_node("RobotControllerNode")
    robot_controller = RobotController()
    arm_init_success = robot_controller.arm_controller.init_arm_status()
    hand_init_success = robot_controller.hand_controller.init_hand_status()
    # robot_controller.test()

    rospy.spin()

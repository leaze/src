#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

def simple_action(x, y, w):
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    ac.send_goal(goal)
    ac.send_goal(goal)
    rospy.loginfo("Sent goal")
    ac.wait_for_result()
    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal succeeded!")
    else:
        rospy.loginfo("Goal failed with error code: " + str(ac.get_state()))


if __name__ == "__main__":
    rospy.init_node("nav_client")
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac.wait_for_server()  # 等待服务
    goal = MoveBaseGoal()  # 目标点
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -3.0
    goal.target_pose.pose.position.y = 2.0
    goal.target_pose.pose.orientation.w = 1.0
    ac.send_goal(goal)
    ac.send_goal(goal)
    rospy.loginfo("Sent goal")
    ac.wait_for_result()
    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal succeeded!")
    else:
        rospy.loginfo("Goal failed with error code: " + str(ac.get_state()))
    simple_action(0.0, 0.0, 1.0)
    time.sleep(5)
    simple_action(2.0, -2.0, 1.0)
    time.sleep(5)
    simple_action(0.0, 0.0, 1.0)
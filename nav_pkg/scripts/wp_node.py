#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from std_msgs.msg import String

# 定义导航点的列表
waypoints = ["1", "2", "3", "4"]
current_index = 0


def NavResultCallback(msg):
    rospy.logwarn("导航结果 = %s", msg.data)


if __name__ == "__main__":
    rospy.init_node("wp_node")

    navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=1)
    res_sub = rospy.Subscriber(
        "/waterplus/navi_result", String, NavResultCallback, queue_size=1
    )

    rospy.sleep(1)

    # 发布第一个导航点
    rospy.logwarn("发布第一个导航点: %s", waypoints[current_index])
    navi_msg = String()
    navi_msg.data = "1"
    navi_pub.publish(navi_msg)
    current_index += 1

    rospy.sleep(20)
    # 发布第二个导航点
    rospy.logwarn("发布第二个导航点: %s", waypoints[current_index])
    # navi_msg = String()
    navi_msg.data = "2"
    navi_pub.publish(navi_msg)
    current_index += 1

    rospy.sleep(20)
    # 发布第三个导航点
    rospy.logwarn("发布第三个导航点: %s", waypoints[current_index])
    # navi_msg = String()
    navi_msg.data = "3"
    navi_pub.publish(navi_msg)
    current_index += 1

    rospy.sleep(20)
    # 发布第四个导航点
    rospy.logwarn("发布第四个导航点: %s", waypoints[current_index])
    # navi_msg = String()
    navi_msg.data = "4"
    navi_pub.publish(navi_msg)

    rospy.spin()
    rospy.logwarn("导航结束")

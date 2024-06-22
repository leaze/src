#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from std_msgs.msg import String

navi_msg = String()
navi_msg.data = "1"


def NavResultCallback(msg):
    rospy.logwarn("导航结果 = %s", msg.data)
    print(navi_msg.data)
    if navi_msg.data == "5":
        # navi_msg.data == "1"
        return
    else:
        navi_msg.data = str(int(navi_msg.data) + 1)
    navi_pub.publish(navi_msg)


if __name__ == "__main__":
    rospy.init_node("wp_node")

    res_sub = rospy.Subscriber(
        "/waterplus/navi_result", String, NavResultCallback, queue_size=10
    )
    navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10)
    rospy.sleep(1)
    navi_pub.publish(navi_msg)
    rospy.spin()
    rospy.logwarn("导航结束")

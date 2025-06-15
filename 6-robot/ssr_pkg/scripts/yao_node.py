#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("yao_node")
    rospy.logwarn("yao_node is running")
    pub = rospy.Publisher("gie_gie_dai_wo", String, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.logwarn("i will sent msgs")
        msg = String()
        msg.data = "please get in the car"
        pub.publish(msg)
        rate.sleep()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("chao_node")
    rospy.logwarn("chao_node is running")
    pub = rospy.Publisher("kuai_shang_che_kai_hei_qun", String, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.logwarn("i will sent msgs")
        msg = String()
        msg.data = "guofu machao daifei"
        pub.publish(msg)
        rate.sleep()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from std_msgs.msg import String


def chao_callback(msg):
    rospy.loginfo(msg.data)


def yao_callback(msg):
    rospy.loginfo(msg.data)


if __name__ == "__main__":
    rospy.init_node("ma_node")
    sub = rospy.Subscriber("kuai_shang_che_kai_hei_qun", String, chao_callback)
    sub2 = rospy.Subscriber("gie_gie_dai_wo", String, yao_callback)
    rospy.spin()

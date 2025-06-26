#!/usr/bin/env python
import rospy
from bodyctrl_msgs.msg import MotorStatusMsg
def callback(data):
    rospy.loginfo("/arm/status:\n %s", data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/arm/status', MotorStatusMsg, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

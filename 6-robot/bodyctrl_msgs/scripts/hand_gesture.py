#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class GestureCommander:
    def __init__(self):
        rospy.init_node('gesture_commander')
        self.gesture_pub = rospy.Publisher('/inspire_hand/cmd_gesture', String, queue_size=10)
        
        rospy.loginfo("Gesture commander ready. Available gestures: fist, open_hand, ok_sign, scissors, thumbs_up")
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                gesture = input("Enter gesture command (or 'exit' to quit): ")
                
                if gesture.lower() == 'exit':
                    rospy.signal_shutdown("User requested exit")
                    break
                
                # 发送手势命令
                cmd_msg = String()
                cmd_msg.data = gesture
                self.gesture_pub.publish(cmd_msg)
                rospy.loginfo(f"Gesture command sent: {gesture}")
                
            except Exception as e:
                rospy.logerr(f"Error processing command: {str(e)}")

if __name__ == '__main__':
    commander = GestureCommander()
    commander.run()
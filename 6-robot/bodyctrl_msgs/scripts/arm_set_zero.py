#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from bodyctrl_msgs.msg import MotorStatusMsg
class ZeroCommandPublisher:
    def __init__(self):
        rospy.init_node('zero_command_publisher')
        self.pub = rospy.Publisher('/arm/cmd_set_zero', String, queue_size=10)
        rospy.loginfo("Zero Command Publisher ready. Use commands: 'left_arm', 'right_arm', 'both_arms', or individual joints like '11,15,21'")
    
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                command = input("Enter zeroing command (or 'exit' to quit): ")
                
                if command.lower() == 'exit':
                    rospy.signal_shutdown("User requested exit")
                    break
                
                # 验证命令格式
                if self.validate_command(command):
                    cmd_msg = String()
                    cmd_msg.data = command
                    self.pub.publish(cmd_msg)
                    rospy.loginfo(f"Sent command: {command}")
                else:
                    rospy.logwarn("Invalid command format. Use: left_arm, right_arm, both_arms, or joint IDs like 11,12,13")
                    
            except Exception as e:
                rospy.logerr(f"Error processing command: {str(e)}")
    
    def validate_command(self, command):
        # 检查预定义命令
        predefined = ["left_arm", "right_arm", "both_arms"]
        if command in predefined:
            return True
        
        # 检查关节ID列表
        try:
            ids = command.split(',')
            for joint_str in ids:
                joint_id = int(joint_str.strip())
                # 检查是否在有效范围内（左臂11-17，右臂21-27）
                if not (11 <= joint_id <= 17 or 21 <= joint_id <= 27):
                    return False
            return True
        except:
            return False

if __name__ == '__main__':
    try:
        publisher = ZeroCommandPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
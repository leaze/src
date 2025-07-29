#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   keyboard_ctrl.py
@Time    :   2025/07/26 13:09:55
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
import rospy
import threading
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import TwistStamped  # 修改为使用 TwistStamped

class G1ControlSystem:
    def __init__(self):
        self.gazebo_process = None
        self.ctrl_process = None
        self.vel_publisher = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)  # 修正为 TwistStamped
        self.is_walking = False
        self.current_vel = TwistStamped()
        self.key_listener_active = True
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True

    def keyboard_listener(self):
        """使用getch实现的键盘监听"""
        import termios, tty, sys
        
        print("\n键盘控制已激活：")
        print("W/S: 前进/后退")
        print("A/D: 左移/右移")
        print("Q/E: 左转/右转")
        print("空格: 停止运动")
        print("按 ESC 退出程序")
        
        # 保存终端设置
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while self.is_walking and self.key_listener_active:
                # 重置速度
                self.current_vel.twist.linear.x = 0
                self.current_vel.twist.linear.y = 0
                self.current_vel.twist.angular.z = 0
                
                # 设置头信息
                self.current_vel.header.stamp = rospy.Time.now()
                self.current_vel.header.frame_id = "base_link"
                
                # 读取按键
                ch = sys.stdin.read(1)
                
                # 处理按键
                if ch == 'w':
                    self.current_vel.twist.linear.x = 0.05
                elif ch == 's':
                    self.current_vel.twist.linear.x = -0.05
                elif ch == 'a':
                    self.current_vel.twist.linear.y = 0.05
                elif ch == 'd':
                    self.current_vel.twist.linear.y = -0.05
                elif ch == 'q':
                    self.current_vel.twist.angular.z = 0.1
                elif ch == 'e':
                    self.current_vel.twist.angular.z = -0.1
                elif ch == ' ':  # 空格键
                    self.current_vel.twist.linear.x = 0
                    self.current_vel.twist.linear.y = 0
                    self.current_vel.twist.angular.z = 0
                elif ord(ch) == 27:  # ESC键
                    self.is_walking = False
                    self.key_listener_active = False
                    print("退出程序")
                    break
                
                # 发布速度指令
                self.vel_publisher.publish(self.current_vel)
                time.sleep(0.05)
                
        finally:
            # 恢复终端设置
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def cleanup(self):
        """清理资源"""
        print("\n清理资源...")
        self.is_walking = False
        self.key_listener_active = False
        
        if self.ctrl_process and self.ctrl_process.isalive():
            self.ctrl_process.close()
        
        if self.gazebo_process:
            self.gazebo_process.terminate()
        
        rospy.signal_shutdown("程序结束")
    
    def run(self):
        try:
            # 初始化ROS节点
            rospy.init_node('g1_control_node', anonymous=True)
            rospy.on_shutdown(self.cleanup)
            self.is_walking = True
            
            # 启动键盘监听
            self.key_thread.start()
            # 等待控制器退出
            while self.is_walking:
                time.sleep(1)
                
        except Exception as e:
            print(f"发生错误: {e}")
        finally:
            self.cleanup()

if __name__ == "__main__":
    control_system = G1ControlSystem()
    control_system.run()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from nav_msgs.msg import OccupancyGrid


if __name__ == "__main__":
    rospy.init_node("map_pub_node")
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
    while not rospy.is_shutdown():
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.info.origin.position.x = -1  # 相对原点X的偏移量
        msg.info.origin.position.y = -1  # 相对原点Y的偏移量
        msg.info.origin.position.z = 0  # 相对原点Z的偏移量
        msg.info.resolution = 1.0  # 地图的分辨率

        msg.info.width = 4  # 地图的宽度
        msg.info.height = 2  #  地图的高度
        msg.data = [0] * msg.info.width * msg.info.height
        msg.data[0] = 100
        msg.data[1] = 100
        msg.data[2] = 0
        msg.data[3] = -1
        pub.publish(msg)
        rospy.sleep(1)

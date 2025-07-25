# rosbridge

<https://zhuanlan.zhihu.com/p/1898754514762313760>

```shell

# 环境安装
sudo apt install ros-${ROS_DISTRO}-rosbridge-server -y
pip3 install websockets -i https://pypi.tuna.tsinghua.edu.cn/simple
# 1 ros1作为rosbridge-server
# ros1 host
roslaunch rosbridge_server rosbridge_websocket.launch
# ros2 client
python3 pub_msg.py  # 发布消息
python3 sub_msg.py  # 订阅消息
# 2 ros2作为rosbridge-server
# ros2 host
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# ros1 client
python3 pub_msg.py  # 发布消息
python3 sub_msg.py  # 订阅消息
```

## nav2 rosbridge

```shell
# 1 ros2作为rosbridge-server
# ros2 host
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
python3 ur_robot/navigation/navigation_server.py
python3 ur_robot/navigation/navigation_client.py
# ros1 client

```

## moveit2 rosbridge

```shell
# ros2 as rosbridge-server
# ros2 host
ros2 launch dual move_group.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
python3 ur_robot/moveit2/ik_service_dual.py
# ros1 client
python3 ur_robot/rosbridge/pub_joints.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0
```

## moveit2 planning

```shell
# ros1 做为 rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=9090
# ros2 做为 Rossbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9091
```

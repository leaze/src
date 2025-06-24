# rosbridge

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

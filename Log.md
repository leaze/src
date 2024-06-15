# 启动Cat_kin

+ source /home/gym/code/catkin_ws/devel/setup.bash
+ roslaunch wpr_simulation wpb_simple.launch

## 速度控制

rosrun rqt_robot_steering rqt_robot_steering

## 在WSL中添加代理

```bash
git config --global http.proxy http://127.0.0.1:7890
git config --global https.proxy https://127.0.0.1:7890
```

## Terminator

+ sudo apt install terminator
+ ctrl+shift+E, ctrl+shift+O

## Node(节点)

1. 使用catkin_create_pkg创建一个软件包
2. 在软件包的src文件夹下创建一个节点的cpp源码文件
3. 在节点的源码文件中include包含ROS的头文件
4. 构建一个main函数, 并在函数的开头执行ros::inti()
5. 构建while循环, 循环条件为ros::ok
6. 在CMakeLists.txt中设置节点源码的编译规则
7. 编译运行

## Topic(话题)

1. 一个ROS节点网络中, 可以同时存在**多个**话题
2. 一个话题可以有**多个**发布者, 也可以有多个订阅者
3. 一个节点可以对**多个**话题进行订阅, 也可以发布**多个**话题
4. 不同的**传感器**小气通常会拥有**各自独立**话题名称, 每个话题只有**一个**发布者
5. 机器人**速度指令话题通**常会有多个发布者, 但是同一时间只能有一个发言人

## Message(消息)

1. 确定**话题名称**和**消息类型**
2. 在代码文件中include**消息类型**对应的**头文件**
3. 在main函数中**通过NodeHandler**大管家**发布**一个话题并得到消息发送对象
4. **生成**要发送的**消息包**并进行发送数据的**赋值**
5. 调用消息发送对象的**publish()函数**将消息包**发送**到话题当中
6. 常用工具
    1. rostopic list
        + 列出当前系统中所有活跃着的话题
    2. rostopic echo 主题名称
        + 显示指定话题中发送的消息包内容
    3. rostopic hz 主题名称
        + 统计指定话题中消息包发送频率
    4. rqt_graph
        + 图像化显示当前系统活跃的节点以及节点间的话题通讯关系

## Subscriber(订阅者)

1. 确定**话题名称**和**消息类型**
2. 在代码文件中include**消息类型**对应的**头文件**
3. 在main函数中**通过NodeHandler**大管家**订阅**一个话题并得到消息订阅对象
4. 定义一个**回调函数**，对接收到的**消息包**进行处理
5. main函数中需要执行**ros::spinOnce()**, 让回调函数能够响应接收到的消息包
6. 常用工具
    1. rqt_graph
        + 图像化显示当前系统活跃的节点以及节点间的话题通讯关系

## launch文件启动节点

```xml
<launch>
    <node pkg= "ssr_pkg" type="yao_node" name="yao_node"/>
    <node pkg= "ssr_pkg" type="chao_node" name="chao_node" launch-prefix="gnome-terminal -e"/>
    <node pkg= "atr_pkg" type="ma_node" name="ma_node" output="screen"/>
</launch>
```


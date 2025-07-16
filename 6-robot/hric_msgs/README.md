# hric_msgs

## SetMotionMode服务

```shell
catkin_create_pkg hric_msgs std_msgs message_generation message_runtime
cd hric_msgs
mkdir srv
touch srv/SetMotionMode.srv
code srv/SetMotionMode.srv
# #请求字段
# #导航之前请求walk模式，导航结束请求stand模式
# uint8 walk_mode_request #请求切换的运动模式-> 0: start, 1: stop, 2: zero, 3: stand, 4: walk
# bool is_need_swing_arm 
# ----
# #回复字段
# bool success #表示服务调用是否成功
# uint32 error_code #错误码
code package.xml
# <build_depend>message_generation</build_depend>
# <exec_depend>message_runtime</exec_depend>
```

## MotionStatus消息

```shell
mkdir msg
touch msg/MotionStatus.msg
# std_msgs/Header header
# geometry_msgs/Twist velocity
# uint8 walk_mode
# bool is_console_control
# bool is_swing_arm
# uint32 error_code
# #错误码
# uint32 NONE=0
# uint32 UNKNOWN=400
```

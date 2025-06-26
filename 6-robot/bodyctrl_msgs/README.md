# bodyctrl_msgs

```shell
catkin_create_pkg bodyctrl_msgs std_msgs message_generation message_runtime
cd bodyctrl_msgs
mkdir msg
touch msg/MotorStatus.msg
code msg/MotorStatus.msg
# # 单个电机状态
# uint16 name   # 电机标识（MotorName）
# float32 pos   # 位置（rad）
# float32 speed # 速度（rad/s）
# float32 current    # 电流（A）
# float32 temperature  # 温度（℃）
# uint32 error   # 错误码
touch msg/MotorStatusMsg.msg
code msg/MotorStatusMsg.msg
# # 多个电机状态消息
# std_msgs/Header header
# MotorStatus[] status  # 电机状态数组
code package.xml
# <build_depend>message_generation</build_depend>
# <exec_depend>message_runtime</exec_depend>
```

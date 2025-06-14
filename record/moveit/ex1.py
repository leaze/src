#!/usr/bin/env python
#coding=utf-8
## BEGIN_SUB_TUTORIAL imports
##
## 为了使用Python MoveIt! 接口，我们需要导入 "moveit_commander" 命名空间。
## 这个命名空间提供了 "MoveGroupCommander"类、"PlanningSceneInterface"类
## 和 "RobotCommander"类（后文会详细讲到）。
## 
## 我们也需要导入 "rospy"模块和一些用到的消息类型：
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  本函数提供了一种测试方法，用以测试 actual 的值是否在 goal 对应值的公差范围内。
  @param: goal       目标参数。浮点型列表、Pose 类型或 PoseStamped 类型消息
  @param: actual     测试参数。浮点型列表、Pose 类型或 PoseStamped 类型消息
  @param: tolerance  公差范围。浮点数
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## 首先，初始化 "moveit_commander" 和 "rospy" 节点:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## 实例化一个 "RobotCommander" 对象. 该对象是机器人与外界的接口：
    robot = moveit_commander.RobotCommander()

    ## 实例化一个 "PlanningSceneInterface" 对象。这个对象是机器人与周围环境的接口:
    scene = moveit_commander.PlanningSceneInterface()


    ## 实例化一个 "MoveGroupCommander" 对象，这是机器人关节组的接口。在本例中，
    ## 特指 panda 机械臂的关节组，因此我们将 group_name 赋值为"panda_arm"。
    ## 如果你使用的是其它的机器人，就应该把值修改为你的机器人对应的运动组。
    ## 这个接口可以用来规划和执行运动。
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## 我们创建一个名为 “DisplayTrajectory” 的发布者，稍后用来发布机器人轨迹，传递给RViz用以可视化：
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## 获取基本信息
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # 获取机器人参考坐标系的名称：
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    # 获取规划组末端执行器link的名称：
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # 获取机器人所有规划组的名称表：
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # 为了调试需要，有时需要获取机器人当前的整体状态：
    print("============ Printing robot state")
    print(robot.get_current_state())

    ## END_SUB_TUTORIAL

    # 其他变量
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## 运动到关节空间中的目标位置：
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## Panda 的零位是一个奇异点,因此我们首先将其移动到一个更好的位置上。
    ## （什么是奇异点？见<https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>）
    # 我们从运动组（group）获取当前的关节角度，再对其进行一些调整：
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # go 命令可以用关节角度、空间位姿来调用，当已经为运动组设置好目标
    # 位置时，也可以不传递任何参数。
    group.go(joint_goal, wait=True)

    # 调用 stop() 命令以确认是否还有未完成的运动。
    group.stop()

    ## END_SUB_TUTORIAL

    # 测试:
    # 注意，由于本节代码不会包含在教程中，所以我们使用类变量
    # 而不是复制的状态变量。
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## 运动到笛卡尔空间中的目标位置：
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## 我们能够为运动组的末端执行器指定一个笛卡尔位置并进行规划：
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    ## 现在，我们调用规划器计算路径并执行：
    plan = group.go(wait=True)
    # 调用 stop() 命令以确认是否还有未完成的运动。
    group.stop()
    # 在规划完成后，清除目标位姿总是有益的。
    # 注意：没有任何函数与 clear_joint_value_targets() 等价。
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # 测试:
    # 注意，由于本节代码不会包含在教程中，所以我们使用类变量
    # 而不是复制的状态变量。
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## 笛卡尔空间路径
    ## ^^^^^^^^^^^^^^^
    ## 你可以直接通过指定一系列的路径点，来为末端执行器规划
    ## 一个笛卡尔空间路径：
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # 首先向上方 (z)
    wpose.position.y += scale * 0.2  # 和侧方运动 (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # 然后进行前/后运动 (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # 最后再进行侧方运动 (y)
    waypoints.append(copy.deepcopy(wpose))

    # 我们想要在1厘米的分辨率内插值笛卡尔坐标路径，因此我们在笛卡尔
    # 坐标平移中指定0.01作为步长。我们将禁用跳转阈值，设置为0.0:
    # group_name = "panda_arm"
    # group = moveit_commander.MoveGroupCommander(group_name)
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # 路径点
                                       0.01,        # 步长
                                       False)         # 跳转阈值（jump_threshold）

    # 注意：在这里，我们只是进行规划，还没有用 move_group 来实际驱动机器人。
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## 显示一条轨迹
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## 你可以使用RViz来进行规划（又名轨迹）的可视化。但这里的 
    ## group.plan() 方法可以自动进行可视化，因此就没有必要了。
    ##
    ## "DisplayTrajectory" 消息有两个主要的值：trajectory_start 和 trajectory。
    ## 我们使用当前机器人状态填充trajectory_start，以便复制任何 AttachedCollisionObjects，
    ## 并将我们的轨迹添加到trajectory中。
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # 发布消息
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## 执行一个规划
    ## ^^^^^^^^^^^^^^^^
    ## 如果想要机器人根据已经规划好的路径运动，使用 execute 函数：
    group.execute(plan, wait=True)

    ## **注意:** 机器人当前的关节位置必须在 RobotTrajectory 的
    ## 第一个路径点的公差范围内，否则 execute() 函数将会失败。
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## 确定已经收到碰撞更新
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 如果Python节点在发布碰撞对象更新消息之前完结（die），则消息
    ## 可能丢失，箱子将不会出现。为了确保进行了更新，我们要等到看到 
    ## "get_known_object_names()" 和 "get_known_object_names()" 
    ## 列表中反映的更改。出于本教程的目的，我们在规划场景中添加、删除、
    ## 附加或分离对象后调用此函数。然后，我们等待更新完成或 “timeout” 秒过去。
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # 测试箱子是否已经附着到对象上。
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # 测试箱子是否在场景中。
      # 注意：附着箱子将会把它从 known_objects 移除。
      is_known = box_name in scene.get_known_object_names()

      # 测试是否已经达到期望状态
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # 休眠，留时间给处理器上的其他线程
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # 如果我们退出while循环而不返回，那么我们就超时了
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## 添加对象到规划场景中
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 首先，我们在规划场景的左边手指处创建一个箱子：
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # 这里将本地变量复制回类变量。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## 把对象附着到机器人上：
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 接下来，我们将把箱子附着在 Panda 的手腕上。先操纵物体，直到机器人能够触摸到物体，
    ## 而规划场景不将接触报告为碰撞即可。通过向 "touch_links" 数组中添加链接名称，我们
    ## 告诉规划场景忽略这些 link 和箱子之间的碰撞。对于 Panda 机器人，我们设置 "grasping_group = 'hand'"。
    ## 如果您使用的是另一个机器人，您应该将此值更改为您的末端执行器组名称。
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # 等待规划场景的更新
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## 从机器人上解除对象
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 我们也能够从规划场景中解除并移除对象：
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # 等待规划场景的更新
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # 这里将类变量复制到本地变量，可以让本教程显得更加清晰。
    # 但在实践中，除非有充分的理由，否则应该直接使用类变量。
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## 从规划场景中移除对象
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 我们从世界中移除箱子：
    scene.remove_world_object(box_name)

    ## **注意:** 要移除对象，对象首先应当被解除。
    ## END_SUB_TUTORIAL

    # 等待规划场景的更新
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print("============ 按 `Enter` 开始设置 moveit_commander (按 ctrl-d 退出) ...")
    tutorial = MoveGroupPythonIntefaceTutorial()

    # print "============ 按 `Enter` 移动到一个关节空间的目标位置 ..."
    # tutorial.go_to_joint_state()

    # print "============ 按 `Enter` 移动到一个笛卡尔空间的目标位置 ..."
    tutorial.go_to_pose_goal()

    # print "============ 按 `Enter` 规划并演示一个笛卡尔空间路径 ..."
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    # print "============ 按 `Enter` 演示一个存储好的路径（这将重新演示一遍笛卡尔路径） ..."
    tutorial.display_trajectory(cartesian_plan)

    # print "============ 按 `Enter` 执行存储好的路径 ..."
    tutorial.execute_plan(cartesian_plan)

    # print "============ 按 `Enter` 向规划场景中添加一个箱子（未附着状态的箱子为绿色） ..."
    tutorial.add_box()

    # print "============ 按 `Enter` 将箱子附着在 Panda 机器人上（附着状态的箱子为紫色） ..."
    tutorial.attach_box()

    # print "============ 按 `Enter` 使机器人附带着箱子规划并执行一个路径 ..."
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan)

    # print "============ 按 `Enter` 从 Panda 机器人上解除箱子（未附着状态的箱子为绿色） ..."
    tutorial.detach_box()

    # print "============ 按 `Enter` 从规划场景中移除箱子 ..."
    # tutorial.remove_box()

    # print "============ Python 教程演示结束！"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
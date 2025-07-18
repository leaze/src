import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from copy import deepcopy

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_fk_demo', anonymous=True)
robot= moveit_commander.RobotCommander()

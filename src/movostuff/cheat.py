import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)
robot = moveit_commander.RobotCommander()
print(robot.get_group_names())

group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)

waypoints = []
scale = 1

wpose = group.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)

wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                                                              0.01,        # eef_step
                                                                                                                 0.0)         # jump_threshold

# Note: We are just planning, not asking move_group to actually move the robot yet:
group.execute(plan)

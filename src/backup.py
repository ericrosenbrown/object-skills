import sys
import random
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionFK
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('microwave_moveit_node',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "door_plan"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

planning_frame = group.get_planning_frame()
#print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
#print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
#print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
#print "============ Printing robot state"
#print robot.get_current_state()
#print ""


joint_goal = group.get_current_joint_values()
#print(joint_goal)
joint_goal[0] = 0.8#random.uniform(-0.3,0.3)

group.go(joint_goal, wait=True)

plan = rospy.wait_for_message('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory) #ideally, rather than listen from this topic for the plan, we would just directly access it from the scene/group, but I am not sure how to do this. come back to later

jt = plan.trajectory[0].joint_trajectory #plan for the joint trajectory!
print("joint plan!")
#print(len(plan.trajectory))
#print(jt.points[0].positions[0]) #first index is which point, second index just gets joint value out of tuple
print(len(jt.points))
print("end joint plan!")

fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

ee_poses = []
for i in range(len(jt.points)): #loop through the plan
    r = moveit_commander.RobotState()

    header = Header(0,rospy.Time.now(),"/base_link")
    joint_names = jt.joint_names
    joint_positions = [jt.points[i].positions[0]]
    fkIn = ["base_link","door","handle"]
    r.joint_state.name = joint_names
    r.joint_state.position = joint_positions
    #rospy.loginfo(fk(header,fkIn,r))
    #print("hey")
    #print(r)
    a = fk(header,fkIn,robot.get_current_state())
    #print(a.pose_stamped[-1]) #print fk pose, -1 because we only care about the fk of the last link, handle.
    print(a.pose_stamped)
    ee_poses.append(a.pose_stamped[-1].pose)

#print(ee_poses[0])
#print(ee_poses[-1])

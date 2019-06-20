import sys
import time
import pickle
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

def generate_pose_stamped_msg(position, orientation):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = '/odom'
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]
    return(pose)

def generate_pose_msg(position, orientation):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return(pose)


def generate_right_arm_plan(goal_pose):
    group_arms.set_pose_target(goal_pose, "right_ee_link")    
    plan = group_arms.plan()
    if not plan.joint_trajectory.joint_names:
        print('Plan failed :(')
        return
    plan_publisher.publish(plan)
    return(plan)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('microwave_movo_node', anonymous=True)

robot = moveit_commander.RobotCommander()

group_arms = moveit_commander.MoveGroupCommander('upper_body')

plan_publisher = rospy.Publisher('/movo_moveit/motion_plan', RobotTrajectory, queue_size=1)

#orientation2 = [-0.98, -0.15, 0, 0.1]
#orientation = [0,0,0,1]
#Actually making a plan
origin_pose = [0.8, -0.1, 0.9]
#origin_pose = [0.9, -0.1, 1.0]

microwave_poses = pickle.load(open("real2_microwave_poses.p", "rb"))
#calculate differences between loaded microwave_poses
displacements = []
for i in range(1,len(microwave_poses)):
    m1 = microwave_poses[i].position
    m2 = microwave_poses[i-1].position
    d = [m1.x-m2.x,m1.y-m2.y,m1.z-m2.z]
    displacements.append(d)
#print(displacements)

#Now apply displacements to origin pose to calculate actual EE poses to go to
actual_poses = [origin_pose]
for disp in displacements:
    new_pose = [j+i for i,j in zip(actual_poses[-1],disp)]
    actual_poses.append(new_pose)
#print(actual_poses)

actual_orientations = []
for p in microwave_poses:
    x = p.orientation.x
    y = p.orientation.y
    z = p.orientation.z
    w = p.orientation.w
    actual_orientations.append([x,y,z,w])

#may need to reverse depending on the situation, this should all be dealt with once we use /map frame for object too
#actual_poses.reverse()
#actual_orientations.reverse()

#print(actual_orientations)

#Now loop through and apply poses
"""
starting_pose = generate_pose_stamped_msg(origin_pose,actual_orientations[0])
plan = generate_right_arm_plan(starting_pose)
group_arms.execute(plan)
print("moved to start!")
"""


waypoints = [generate_pose_msg(origin_pose,actual_orientations[0])]

for p,o in zip(actual_poses,actual_orientations):
    #print(p,o)
    pose = generate_pose_msg(p,o)
    waypoints.append(pose)

right_arm = moveit_commander.MoveGroupCommander('right_arm')

print("moving to start pose")
(plan, fraction) = right_arm.compute_cartesian_path(waypoints[:1], 0.01, 0.0)
print(right_arm.execute(plan))
print("moving in 3 seconds")
rospy.sleep(3)
(plan, fraction) = right_arm.compute_cartesian_path(waypoints, 0.01, 0.0)
print(fraction)
print(right_arm.execute(plan))
#print(robot.get_current_state())

#group_arms.execute(plan)


    #plan = generate_right_arm_plan(pose)
    #group_arms.execute(plan)


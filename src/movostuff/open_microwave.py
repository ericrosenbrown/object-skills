import sys
import pickle
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseStamped

def generate_pose_msg(position, orientation):
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
print(displacements)

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

print(actual_orientations)


#Now loop through and apply poses
for p,o in zip(actual_poses,actual_orientations):
    print(p,o)
    pose = generate_pose_msg(p,o)
    plan = generate_right_arm_plan(pose)
    group_arms.execute(plan)
"""

#above code is everything. for now, just plan first and last pose
z = zip(actual_poses,actual_orientations)
print("first pose")
pose = generate_pose_msg(z[0][0],z[0][1])
plan = generate_right_arm_plan(pose)
group_arms.execute(plan)
print("last pose")

pose = generate_pose_msg(z[-1][0],z[-1][1])
plan = generate_right_arm_plan(pose)
group_arms.execute(plan)
"""


"""
#test pose
test1 = [0.9, 0.2, 1.0]
test2 = [0.9, -0.1, 1.0]

position1=[-0.5,0.1,0.0]
position2=[-0.501899760638,0.0899812773384, 0.0]

d1 = [j-i for i,j in zip(position1,position2)]
print(d1)
test3 = [j+i for i,j in zip(test1,d1)]

orientation = [-0.98, -0.15, 0, 0.1]

pose = generate_pose_msg(test1,orientation)
plan = generate_right_arm_plan(pose)
print(plan)
group_arms.execute(plan)


pose = generate_pose_msg(test3,orientation)
plan = generate_right_arm_plan(pose)
print(plan)
group_arms.execute(plan)

microwave_poses = pickle.load(open("microwave_poses.p", "rb"))
print(microwave_poses)
"""

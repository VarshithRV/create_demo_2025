import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group_name = "panda_arm"
# this is the interface to command a group of joints, i.e. move group,
# there can be more than one such object in the same node to command multiple groups at the same time
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# get the planning frame
planning_frame = move_group.get_planning_frame()
print("Planning frame : %s" %planning_frame)

# get the end effector link
# im guessing that this configuration for the move_group id is set in the 
# moveit configuration file
eef_link = move_group.get_end_effector_link()
print("End effector link : %s" % eef_link)

# get all the group names in the robot
group_names = robot.get_group_names()
print("All planning groups : %s" %group_names)

# print the entire state of the robot
print("Robot state ")
print(robot.get_current_state())

# # tau is one turn, 360
# joint_goal = move_group.get_current_joint_values()
# joint_goal[0] = 0
# joint_goal[1] = -tau/8
# joint_goal[2] = 0
# joint_goal[3] = -tau/4
# joint_goal[4] = 0
# joint_goal[5] = tau/6
# joint_goal[6] = 0

# # go can be called with joint values, poses, or without any
# # parameters if there is already a pose or joint target for 
# # the group set
# move_group.go(joint_goal, wait=True) # wait false will get
# # keep the code running async and not wait for the result,
# # function "go" also returns a truth value for success

# # make sure that there is no residual movement
# move_group.stop()

# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 1
# pose_goal.position.x = 0.4
# pose_goal.position.y = 0.1
# pose_goal.position.z = 0.4

# # setting the target
# move_group.set_pose_target(pose_goal)

# success =  move_group.go(wait=True)
# move_group.stop()
# move_group.clear_pose_targets()

# now we do cartesian paths using waypoints
waypoints = []
wpose = move_group.get_current_pose().pose
scale  = 1
wpose.position.z -= scale*0.1 # move up (z)
wpose.position.y -= scale*0.2 # sideways (y)
waypoints.append(copy.deepcopy(wpose))

# second move forwards in x
wpose.position.x += scale*0.1
waypoints.append(copy.deepcopy(wpose))

# third move sideways (y)
wpose.position.y -=  scale*0.1
waypoints.append(copy.deepcopy(wpose))

# this is planning, not execution yet
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0 # waypoints to follow # eef_step and # jump_threshold
)

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# publish
display_trajectory_publisher.publish(display_trajectory)

move_group.execute(plan,wait=True)

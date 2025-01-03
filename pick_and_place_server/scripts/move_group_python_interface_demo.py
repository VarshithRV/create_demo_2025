import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros, tf2_geometry_msgs

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# initialize
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "left_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

print("============ Printing robot state")
print(robot.get_current_state())
print("")

# left_tool0 pose = [
# header: 
#   seq: 0
#   stamp: 
#     secs: 1735812307
#     nsecs: 298247098
#   frame_id: "world"
# pose: 
#   position: motion_planning
#     x: -0.30930782707202664
#     y: 0.29902708101091746
#     z: 0.7277326524907601
#   orientation: 
#     x: 0.21604308098095068
#     y: -0.9763494141190178
#     z: -0.006759413292882849
#     w: 0.0046388621067220415 ]

# create a PoseStamped message in the frame 
pose = geometry_msgs.msg.PoseStamped()
pose.header.frame_id = "world"
pose.pose.position.x = -0.336416748860713
pose.pose.position.y = 0.1941691782500851
pose.pose.position.z = 0.5079139956473084+0.2
pose.pose.orientation.x = -0.9485312182544288
pose.pose.orientation.y = 0.2648719207216197
pose.pose.orientation.z = -0.13743456521576660
pose.pose.orientation.w = 0.10603364510363038

move_group.set_pose_target(pose)
success = move_group.go(wait=True)
move_group.stop()

move_group.clear_pose_targets()

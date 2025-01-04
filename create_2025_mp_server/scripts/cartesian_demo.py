import rospy
import copy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose

def cartesian_path_with_waypoints():
    # Initialize ROS node
    rospy.init_node('cartesian_path_planning', anonymous=True)

    # Create MoveIt interfaces
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "left_arm"  # Replace with your planning group
    move_group = MoveGroupCommander(group_name)

    # Get the current pose of the end effector
    start_pose = move_group.get_current_pose().pose
    waypoints = []

    # Define 4 waypoints relative to the starting pose
    wpose = copy.deepcopy(start_pose)

    # Waypoint 1: Move along the X-axis
    wpose.position.x += 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Waypoint 2: Move along the Y-axis
    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Waypoint 3: Move along the Z-axis
    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Waypoint 4: Move diagonally back towards the starting position
    wpose.position.x -= 0.05
    wpose.position.y -= 0.05
    wpose.position.z -= 0.05
    waypoints.append(copy.deepcopy(wpose))

    # Plan Cartesian path
    eef_step = 0.005  # Step size in meters
    jump_threshold = 0.0  # Disable jump threshold
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step)

    rospy.loginfo(f"Computed path fraction: {fraction}")

    # Check if the path was fully computed
    if fraction > 0.99:
        rospy.loginfo("Executing the Cartesian path...")
        move_group.execute(plan, wait=True)
    else:
        rospy.logwarn("Cartesian path planning failed to compute a valid trajectory.")

    rospy.loginfo("Path execution completed.")

if __name__ == "__main__":
    try:
        cartesian_path_with_waypoints()
    except rospy.ROSInterruptException:
        pass

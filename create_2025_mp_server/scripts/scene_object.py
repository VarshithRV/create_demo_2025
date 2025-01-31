import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

def add_collision_objects(scene):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = -0.21
    box_pose.pose.position.z = 0.1
    box_pose.pose.orientation.w = 1.0
    scene.add_box("wall1", box_pose, size=(0.4, 0.01, 0.2))

    box_pose.pose.position.x = 0.21
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.1
    box_pose.pose.orientation.w = 1.0
    scene.add_box("wall2", box_pose, size=(0.01, 0.4, 0.2))

    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.21
    box_pose.pose.position.z = 0.1
    box_pose.pose.orientation.w = 1.0
    scene.add_box("wall3", box_pose, size=(0.4, 0.01, 0.2))

    box_pose.pose.position.x = -0.21
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.1
    box_pose.pose.orientation.w = 1.0
    scene.add_box("wall4", box_pose, size=(0.01, 0.4, 0.2))
def add_ground(scene):
     # Define the pose and size of the ground plane
    ground_pose = geometry_msgs.msg.PoseStamped()
    ground_pose.header.frame_id = "world"
    ground_pose.pose.orientation.w = 1.0
    ground_pose.pose.position.z = 0.0  # Adjust this if your ground is at another height
    scene.add_plane("ground", ground_pose, normal=(0, 0, 1))

def plan_path(group, target_pose):
    group.set_pose_target(target_pose)
    success,plan,plan_time, error = group.plan()

    # print(plan)
    # print(len(plan))
    return plan  # Ensure only a valid trajectory is returned

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_path_planner', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    rospy.sleep(2)  # Ensure the scene is updated
    add_collision_objects(scene)
    rospy.sleep(2)  # Allow time for collision objects to be added
    add_ground(scene)
    rospy.sleep(2)  # Allow time for collision objects to be added

    # Define common orientation
    orientation = geometry_msgs.msg.Pose()
    orientation.orientation.x= 0.3584616221783295
    orientation.orientation.y= -0.9331510255055198
    orientation.orientation.z= -0.007891893302493867
    orientation.orientation.w= 0.025925798799257518 

    # Define pick and place points
    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x= 0.40091736678970846
    pick_pose.position.y= 0.07709041617034952
    pick_pose.position.z= 0.0465578734476713
    pick_pose.orientation = orientation.orientation

    place_pose = geometry_msgs.msg.Pose()
    place_pose.position.x= 0.07967836612934143
    place_pose.position.y= 0.001687492860993788
    place_pose.position.z= 0.06782958226851464
    place_pose.orientation = orientation.orientation

    while True : 
        rospy.loginfo("Planning path to pick position...")
        pick_plan = plan_path(move_group, pick_pose)
        if pick_plan:
            rospy.loginfo("Executing pick motion...")
            move_group.execute(pick_plan, wait=True)
        else:
            rospy.logwarn("Failed to plan to pick position.")

        rospy.loginfo("Planning path to place position...")
        place_plan = plan_path(move_group, place_pose)
        if place_plan:
            rospy.loginfo("Executing place motion...")
            move_group.execute(place_plan, wait=True)
        else:
            rospy.logwarn("Failed to plan to place position.")

        
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

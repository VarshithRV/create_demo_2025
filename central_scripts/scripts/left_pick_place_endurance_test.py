# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
import actionlib

class MpClass:
    def __init__(self):
        self.left_pick_place_client = actionlib.SimpleActionClient("left_pick_place", PickPlaceAction)
        rospy.loginfo("waiting for server")
        self.left_pick_place_client.wait_for_server()
        rospy.loginfo("server connected")
        # self.right_pick_place_client = actionlib.SimpleActionClient("right_pick_place", PickPlaceAction)
        # self.right_pick_place_client.wait_for_server()

if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()

    ORIENTATION_POSE = PoseStamped()
    ORIENTATION_POSE.pose.orientation.x= -0.9213484323776968
    ORIENTATION_POSE.pose.orientation.y= 0.38857296439791666
    ORIENTATION_POSE.pose.orientation.z= 0.00429333977367731
    ORIENTATION_POSE.pose.orientation.w= 0.010473047682687926

    source = PoseStamped()
    source.pose.position.x= 0.0
    source.pose.position.y= 0.10
    source.pose.position.z= 0.02
    source.pose.orientation=ORIENTATION_POSE.pose.orientation

    destination = PoseStamped()
    destination.pose.position.x= 0.0
    destination.pose.position.y= -0.10
    destination.pose.position.z= 0.02
    destination.pose.orientation= ORIENTATION_POSE.pose.orientation

    rospy.loginfo("Sending pick and place goal")
    pick_place_goal = PickPlaceGoal()
    pick_place_goal.source = source
    pick_place_goal.destination = destination
    i=0
    while i<20:
        mp.left_pick_place_client.send_goal(pick_place_goal)
        mp.left_pick_place_client.wait_for_result()
        pick_place_result = mp.left_pick_place_client.get_result()
        print("Pick and place result : ", pick_place_result.result)
        i+=1
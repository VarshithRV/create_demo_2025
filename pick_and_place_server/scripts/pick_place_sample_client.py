# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from pick_and_place_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
import actionlib

class MpClass:
    def __init__(self):
        self.pick_place_client = actionlib.SimpleActionClient("left_pick_place", PickPlaceAction)
        self.pick_place_client.wait_for_server()


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    
    source = PoseStamped()

    source.pose.position.x = -0.5363869364887379
    source.pose.position.y = 0.16829406964448285
    source.pose.position.z = 0.2098780186920811
    source.pose.orientation.x = 0.9653943031274262
    source.pose.orientation.y = -0.2607581565937913
    source.pose.orientation.z = -0.004191010953791741
    source.pose.orientation.w = 0.0012077607809772351

    destination = PoseStamped()
    destination.pose.position.x = -0.4019202822119746
    destination.pose.position.y = -0.12305345286926832
    destination.pose.position.z = 0.20551334625241705
    destination.pose.orientation.x = 0.9655861251973878
    destination.pose.orientation.y = -0.26005584641028756
    destination.pose.orientation.z = -0.0037183083958979746
    destination.pose.orientation.w = 0.0007521680640413613

    rospy.loginfo("Sending pick and place goal")
    pick_place_goal = PickPlaceGoal()
    pick_place_goal.source = source
    pick_place_goal.destination = destination

    mp.pick_place_client.send_goal(pick_place_goal)
    mp.pick_place_client.wait_for_result()
    pick_place_result = mp.pick_place_client.get_result()
    print("Pick and place result : ", pick_place_result.result)
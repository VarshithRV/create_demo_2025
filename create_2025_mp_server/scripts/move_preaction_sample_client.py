# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from create_2025_mp_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib

class MpClass:
    def __init__(self):
        self.left_move_preaction_client = actionlib.SimpleActionClient("left_move_preaction", MovePreactionAction)
        self.right_move_preaction_client = actionlib.SimpleActionClient("right_move_preaction", MovePreactionAction)
        rospy.loginfo("waiting for server")
        self.left_move_preaction_client.wait_for_server()
        self.right_move_preaction_client.wait_for_server()
        rospy.loginfo("server connected")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()

    rospy.loginfo("Sending move preaction goal to left arm")
    move_preaction_goal = MovePreactionActionGoal()
    mp.left_move_preaction_client.send_goal(move_preaction_goal)
    mp.left_move_preaction_client.wait_for_result()
    move_preaction_result = mp.left_move_preaction_client.get_result()
    print("Move preaction result : ", move_preaction_result.result)
    
    rospy.loginfo("Sending move preaction goal to right arm")
    move_preaction_goal = MovePreactionActionGoal()
    mp.right_move_preaction_client.send_goal(move_preaction_goal)
    mp.right_move_preaction_client.wait_for_result()
    move_preaction_result = mp.right_move_preaction_client.get_result()
    print("Move preaction result : ", move_preaction_result.result)

    print("Done")
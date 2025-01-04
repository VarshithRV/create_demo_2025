import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PointStamped, Pose
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_srvs.srv import SetBool
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceActionGoal, PickPlaceActionResult
from create_2025_mp_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib

class Motion_planner:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.loginfo("Initializing motion planner")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.gripper_client = rospy.ServiceProxy("gripper", SetBool)

        self.pre_action_joint_state = self.move_group.get_current_joint_values()
        self.pre_action_joint_state[0] = -2.815
        self.pre_action_joint_state[1] = -2.240
        self.pre_action_joint_state[2] = 2.320
        self.pre_action_joint_state[3] = -1.657
        self.pre_action_joint_state[4] = -1.566
        self.pre_action_joint_state[5] = 0
        self.waypoints = []

        # create an action server for move preaction
        self.move_preaction_server = actionlib.SimpleActionServer(
            "move_preaction", MovePreactionAction, self.move_preaction_callback, auto_start=False
        )

        self.move_preaction_server.start()

    def move_preaction_callback(self, goal:MovePreactionActionGoal):
        rospy.loginfo("Received move preaction goal")
        success = self.move_to_preaction()
        result = MovePreactionActionResult()
        result.result = success

        if success:
            self.move_preaction_server.set_succeeded(result)
        else:
            self.move_preaction_server.set_aborted(result)

    
    def move_to_preaction(self):
        joint_goal = self.pre_action_joint_state
        rospy.loginfo("Moving to preaction joint state")
        rospy.loginfo("Joint goal : %s", joint_goal)
        try : 
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False
        return True
        
if __name__  == "__main__":
    rospy.init_node("preaction_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        



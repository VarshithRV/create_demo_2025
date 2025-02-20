import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_srvs.srv import SetBool
from create_2025_mp_server_msgs.msg import PoseAction, PoseGoal, PoseResult
import actionlib
from ur_msgs.srv import SetIO

### variable bound for change
# move up or down, set gripper value, time before activating gripper and moving
PICK_PLACE_HEIGHT = 0.3
LOOK_HEIGHT = 0.26

class Motion_planner:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.loginfo("Initializing motion planner")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # get the planning frame
        planning_frame = self.move_group.get_planning_frame()
        print("Planning frame : %s" %planning_frame)

        # get the end effector link
        # im guessing that this configuration for the move_group id is set in the 
        # moveit configuration file
        eef_link = self.move_group.get_end_effector_link()
        print("End effector link : %s" % eef_link)

        # get all the group names in the robot
        group_names = self.robot.get_group_names()
        print("All planning groups : %s" %group_names)

        # print the entire state of the robot
        print("Robot state ")
        print(self.robot.get_current_state())

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.waypoints = []

        # create action server for pick and place
        self.place_server = actionlib.SimpleActionServer(
            "right_place", PoseAction, self.place_callback, auto_start=False
        )

        # subscribe to the right_get_object_locations
        self.right_get_object_locations_service = rospy.ServiceProxy(
            "right_get_object_locations",
            GetObjectLocations
        )

        # create a service client for /right/ur_hardware_interface/set_io
        self.set_io_client = rospy.ServiceProxy("/right/ur_hardware_interface/set_io", SetIO)
        self.set_io_client.wait_for_service()
        self.place_server.start()

    def execute_waypoints(self, waypoints):
        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints : %s", waypoints)

        # plan a cartesian path
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
            # rospy.loginfo("Manually retiming the trajectory with velocity_scaling = 1, acceleration_scaling_factor = 0.5")
            plan=self.move_group.retime_trajectory(self.move_group.get_current_state(),plan,velocity_scaling_factor = 1.0,algorithm="time_optimal_trajectory_generation")

        except Exception as e:
            print(e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing")
        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False

    def place_callback(self, goal:PoseGoal):
        rospy.loginfo("Received pick and place goal")
        end = goal.pose
        success = self.place(end)
        
        # set goal to success
        result = PoseResult()
        result.result = success

        if success:
            self.place_server.set_succeeded(result)
        else:
            self.place_server.set_aborted(result)


    def place(self,end:PoseStamped):
        rospy.loginfo("Started place with end : %s", end)

        pick_place_height = PICK_PLACE_HEIGHT
        
        waypoints = []
        preplace = Pose()
        preplace = copy.deepcopy(end.pose)
        preplace.position.z = pick_place_height
        waypoints.append(copy.deepcopy(preplace))
        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        waypoints = []
        place = copy.deepcopy(end.pose)
        waypoints.append(copy.deepcopy(place))

        self.execute_waypoints(waypoints)
        
        rospy.sleep(1)

        # deactivate the gripper here
        waypoints = []
        rospy.loginfo("Deactivating gripper")
        self.set_io_client(1,12,0)

        rospy.sleep(1)

        waypoints = []
        current_pose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(preplace))
        self.execute_waypoints(waypoints)

        return True

        
if __name__  == "__main__":
    rospy.init_node("right_place_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        



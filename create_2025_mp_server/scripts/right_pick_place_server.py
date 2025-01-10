import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_srvs.srv import SetBool
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceActionGoal, PickPlaceActionResult
import actionlib
from ur_msgs.srv import SetIO

### variable bound for change
# move up or down, set gripper value, time before activating gripper and moving

### CREATE X Y Z LITERALS FOR PADDING between end effector ###
PADDING_X = 0.0
PADDING_Y = 0.0
PADDING_Z = 0.0
#########################################

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
        self.pick_place_server = actionlib.SimpleActionServer(
            "right_pick_place", PickPlaceAction, self.pick_place_callback, auto_start=False
        )

        
        # create a service client for /left/ur_hardware_interface/set_io
        self.set_io_client = rospy.ServiceProxy("/right/ur_hardware_interface/set_io", SetIO)
        self.set_io_client.wait_for_service()
        self.pick_place_server.start()

    def execute_waypoints(self, waypoints):
        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints : %s", waypoints)

        # plan a cartesian path
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
        except Exception as e:
            print(e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing prepick")
        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False

    def pick_place_callback(self, goal:PickPlaceActionGoal):
        rospy.loginfo("Received pick and place goal")
        start = goal.source
        end = goal.destination
        
        success = self.pick_and_place(start, end)
        
        # set goal to success
        result = PickPlaceActionResult()
        result.result = success

        if success:
            self.pick_place_server.set_succeeded(result)
        else:
            self.pick_place_server.set_aborted(result)


    def pick_and_place(self,start:PoseStamped, end:PoseStamped):
        rospy.loginfo("Started pick and place with start : %s and end : %s", start, end)

        start.pose.position.x += PADDING_X
        start.pose.position.y += PADDING_Y
        start.pose.position.z += PADDING_Z

        end.pose.position.x += PADDING_X
        end.pose.position.y += PADDING_Y
        end.pose.position.z += PADDING_Z

        start.pose.orientation.x= -0.35862806853220586
        start.pose.orientation.y= -0.9334403528397598
        start.pose.orientation.z= -0.0036593492588516663
        start.pose.orientation.w= 0.007850179249297016


        end.pose.orientation.x= -0.35862806853220586
        end.pose.orientation.y= -0.9334403528397598
        end.pose.orientation.z= -0.0036593492588516663
        end.pose.orientation.w= 0.007850179249297016

        # start and end pose are configured with 0 linear transformation and fixed orientation

        pick_place_height = 0.4271641690575525

        # plan a cartesian path to pick, prepick -> pick
        waypoints = []
        initial_pose = self.move_group.get_current_pose().pose
        prepick = Pose()
        prepick = copy.deepcopy(start.pose)
        prepick.position.z = pick_place_height
        waypoints.append(copy.deepcopy(initial_pose))
        waypoints.append(copy.deepcopy(prepick))
        
        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        waypoints = []
        initial_pose = self.move_group.get_current_pose().pose
        prepick2 = Pose()
        prepick2 = copy.deepcopy(start.pose)
        prepick2.position.z = pick_place_height - 0.1
        waypoints.append(copy.deepcopy(initial_pose))
        waypoints.append(copy.deepcopy(prepick2))
        
        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        waypoints = []
        pick = copy.deepcopy(start.pose)
        waypoints.append(copy.deepcopy(pick))

        self.execute_waypoints(waypoints) # here it as at pick at the same position as the one provided in the client

        rospy.sleep(1)

        # activate the gripper here
        rospy.loginfo("Activating gripper")
        self.set_io_client(1,12,1)

        rospy.sleep(1)

        # plan cartesian path to pick -> prepick -> preplace
        waypoints = []
        current_pose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(prepick2))
        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        waypoints = []
        current_pose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(prepick))
        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)
        
        waypoints = []
        preplace = Pose()
        preplace = copy.deepcopy(end.pose)
        preplace.position.z = pick_place_height
        waypoints.append(copy.deepcopy(preplace))
        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        waypoints = []
        preplace2 = Pose()
        preplace2 = copy.deepcopy(preplace)
        preplace2.position.z = pick_place_height - 0.1
        waypoints.append(copy.deepcopy(preplace2))
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
        waypoints.append(copy.deepcopy(preplace2))
        self.execute_waypoints(waypoints)

        waypoints = []
        current_pose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(preplace))
        self.execute_waypoints(waypoints)

        return True
        
if __name__  == "__main__":
    rospy.init_node("right_pick_place_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        



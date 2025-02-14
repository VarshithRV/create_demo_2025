import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_srvs.srv import SetBool
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceActionGoal, PickPlaceActionResult
import actionlib
from ur_msgs.srv import SetIO
from std_srvs.srv import TriggerRequest, Trigger
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from geometry_msgs.msg import WrenchStamped, Twist

### variable bound for change
# move up or down, set gripper value, time before activating gripper and moving
FT_SETPOINT = 8.0
ERROR_ALLOWANCE = 1.0
GROUND_CLEARANCE = 0.05
VELOCITY_z = -0.02
P = 1
I = 1
D = 1
PICK_PLACE_HEIGHT = 0.3
LOOK_HEIGHT = 0.23

class Motion_planner:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        self.force_z = None
        self.command_vel = None
        
        rospy.loginfo("Initializing motion planner")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "left_arm"
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
            "left_pick_place", PickPlaceAction, self.pick_place_callback, auto_start=False
        )

        # subscribe to the left_get_object_locations
        self.left_get_object_locations_service = rospy.ServiceProxy(
            "left_get_object_locations",
            GetObjectLocations
        )

        # create a service client for /left/ur_hardware_interface/set_io
        self.set_io_client = rospy.ServiceProxy("/left/ur_hardware_interface/set_io", SetIO)
        self.set_io_client.wait_for_service()
        self.pick_place_server.start()

        # ee twist publisher
        self.twist_command_publisher = rospy.Publisher("/left/twist_controller/command",Twist,queue_size=10)
        
        # service for zeroing ft sensor
        self.zero_ftsensor = rospy.ServiceProxy('/left/ur_hardware_interface/zero_ftsensor', Trigger)
        self.zero_ftsensor.wait_for_service()
        
        # service for controller switching
        self.switch_controller = rospy.ServiceProxy('/left/controller_manager/switch_controller', SwitchController)
        self.switch_controller.wait_for_service()
        
        # subscriber for ft
        rospy.Subscriber("/left/wrench",WrenchStamped,callback=self.wrench_cb)
        rospy.wait_for_message("/left/wrench",WrenchStamped)
        
        rospy.loginfo("All services registered")

    def wrench_cb(self,msg:WrenchStamped):
        self.force_z = msg.wrench.force.z


    def PID(self):
        error = FT_SETPOINT - self.force_z
        if error > ERROR_ALLOWANCE:
            self.command_vel.linear.z = VELOCITY_z
        else :
            self.command_vel.linear.z = 0.0

    def touch_ft_feedback(self):
        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.start_controllers = ["twist_controller"]
        switch_controller_msg.stop_controllers = ["scaled_pos_joint_traj_controller"]
        switch_controller_msg.strictness = switch_controller_msg.STRICT
        switch_controller_msg.start_asap = True

        rospy.loginfo("Switching controller from scaled_pos_joint_traj_controller to twist_controller")
        try:
            switch_controller_response = self.switch_controller(switch_controller_msg)
        except Exception as e:
            rospy.logerr(f"error occurred while switching controllers = {e}")
        if not switch_controller_response.ok:
            rospy.logerr("Switch controller failed to switch to twist controller")
            return None
        rospy.loginfo("Controller switched")

        rate = rospy.Rate(30)
        self.command_vel = Twist()
        rospy.loginfo("Moving downwards now")
        trigger = TriggerRequest()

        rospy.loginfo("Zeroeing ft sensor")
        try:
            zero_ftsensor_response = self.zero_ftsensor(trigger)
            if not zero_ftsensor_response.success :
                rospy.logwarn("Couldn't zero ft sensor beforehand")
        except Exception as e:
            rospy.logerr(f"error occurred while zeroeing ft = {e}")

        i=0

        while not rospy.is_shutdown():
            self.PID()
            self.twist_command_publisher.publish(self.command_vel)
            if self.command_vel.linear.z==0:
                i+=1
                if i>10:
                    break
            rate.sleep()

        switch_controller_msg = SwitchControllerRequest()
        switch_controller_msg.start_controllers = ["scaled_pos_joint_traj_controller"]
        switch_controller_msg.stop_controllers = ["twist_controller"]
        switch_controller_msg.strictness = switch_controller_msg.STRICT
        switch_controller_msg.start_asap = True

        rospy.loginfo("Arrived, giving back control to scaled_pos_joint_traj_controller")
        try:
            switch_controller_response = self.switch_controller(switch_controller_msg)
        except Exception as e:
            rospy.logerr(f"error occurred while switching controllers = {e}")
        if not switch_controller_response.ok:
            rospy.logerr("Switch controller failed to switch to twist controller")
            return None
        rospy.loginfo("Controller switched")
        return True
    
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

        pick_place_height = PICK_PLACE_HEIGHT
        look_height = LOOK_HEIGHT

        # plan a cartesian path to pick, prepick -> pick
        waypoints = []
        initial_pose = self.move_group.get_current_pose().pose
        prepick = Pose()
        prepick = copy.deepcopy(start.pose)
        prepick.position.z = pick_place_height
        prepick.position.y += 0.05 # for the camera to stare at the object
        waypoints.append(copy.deepcopy(initial_pose))
        waypoints.append(copy.deepcopy(prepick))
        
        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        # get a closer look at the object
        waypoints = []
        initial_pose = self.move_group.get_current_pose().pose
        look = prepick
        look.position.z = look_height
        waypoints.append(copy.deepcopy(initial_pose))
        waypoints.append(copy.deepcopy(look))

        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        # call the perception here
        response = GetObjectLocationsResponse()
        response = self.left_get_object_locations_service()
        if response is not None :
            pass
        else :
            rospy.logerr("Second Perception failed, motion plan failed")
            return None
        
        if len(response.result.object_position) > 1 or len(response.result.object_position) ==0:
            rospy.logwarn("Multiple or no objects detected while taking a closer look, might lead to wrong object being picked")
        object_pose = response.result.object_position[0].pose.pose
        rospy.loginfo("Detected object : %s" %response.result.object_position[0].pose.pose)

        # picking the object
        rospy.loginfo("####### Executing pick after second perception ########")
        waypoints  = []
        initial_pose = self.move_group.get_current_pose().pose
        pick = copy.deepcopy(object_pose)
        pick.position.z += GROUND_CLEARANCE
        pick.orientation = start.pose.orientation
        correction = copy.deepcopy(pick)
        correction.position.z = initial_pose.position.z
        #### adding padding here ############################# remove it if when not testing #######
        # pick.position.z = 0.04
        ############################################################################################
        waypoints.append(copy.deepcopy(initial_pose))
        waypoints.append(copy.deepcopy(correction))
        waypoints.append(copy.deepcopy(pick))

        self.execute_waypoints(waypoints)
        rospy.sleep(0.2)

        # touch ft feedback here
        touch_status = self.touch_ft_feedback()
        rospy.loginfo(f"Touch status : {touch_status}")

        rospy.sleep(0.2)
        rospy.loginfo("Activating gripper")
        self.set_io_client(1,12,1)

        rospy.sleep(1)

        waypoints = []
        current_pose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(correction))
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
    rospy.init_node("left_pick_place_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        



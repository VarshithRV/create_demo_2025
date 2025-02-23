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
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceActionGoal, PickPlaceActionResult, SwipeAction, SwipeGoal, SwipeResult
import actionlib    
from ur_msgs.srv import SetIO
from std_srvs.srv import TriggerRequest, Trigger
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from geometry_msgs.msg import WrenchStamped, Twist

### variable bound for change
# move up or down, set gripper value, time before activating gripper and moving
FT_SETPOINT = 2 # force torque set point to indicate pickup
ERROR_ALLOWANCE = 1.0 # force torque condition error allowance, do not use
OBJECT_CLEARANCE = 0.03 # pre force torque based vel controller height for pickup
PRESWIPE_HEIGHT = 0.25 # pre force torque based vel controller height for swiping
VELOCITY_Z = -0.015 # force torque velocity for touch
VELOCITY_X = 0.01 # swipe velocity
PADDING_X = 0.02 # padding for x for swiping area
PADDING_Y = 0.02 # padding for y for swiping area
Y_STEP = 0.02 # steps of y for swiping
P = 1
I = 1
D = 1
PICK_PLACE_HEIGHT = 0.3
LOOK_HEIGHT = 0.23
X_MIN = -0.4 #x min of the field
X_MAX = +0.4 #x max of the field


class Motion_planner:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        self.force_z = None
        self.command_vel = None
        
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
        self.swipe_server = actionlib.SimpleActionServer(
            "right_swipe", SwipeAction, self.swipe_callback, auto_start=False
        )

        # subscribe to the right_get_object_locations
        self.right_get_object_locations_service = rospy.ServiceProxy(
            "right_get_object_locations",
            GetObjectLocations
        )

        # create a service client for /right/ur_hardware_interface/set_io
        self.set_io_client = rospy.ServiceProxy("/right/ur_hardware_interface/set_io", SetIO)
        self.set_io_client.wait_for_service()
        self.swipe_server.start()

        # ee twist publisher
        self.twist_command_publisher = rospy.Publisher("/right/twist_controller/command",Twist,queue_size=10)
        
        # service for zeroing ft sensor
        self.zero_ftsensor = rospy.ServiceProxy('/right/ur_hardware_interface/zero_ftsensor', Trigger)
        self.zero_ftsensor.wait_for_service()
        
        # service for controller switching
        self.switch_controller = rospy.ServiceProxy('/right/controller_manager/switch_controller', SwitchController)
        self.switch_controller.wait_for_service()
        
        # subscriber for ft
        rospy.Subscriber("/right/wrench",WrenchStamped,callback=self.wrench_cb)
        rospy.wait_for_message("/right/wrench",WrenchStamped)
        
        rospy.loginfo("All services registered")

    def wrench_cb(self,msg:WrenchStamped):
        self.force_z = msg.wrench.force.z

    def PID_FT(self):
        error = FT_SETPOINT - self.force_z
        if error > 0:
            self.command_vel.linear.z = VELOCITY_Z
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
            self.PID_FT()
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
    
    def swipe_left(self,X_left):
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
        self.command_vel.linear.x = -VELOCITY_X

        i=0

        while not rospy.is_shutdown():
            
            if self.move_group.get_current_pose().pose.position.x > X_left:
                self.command_vel.linear.x = -VELOCITY_X
            else:
                self.command_vel.linear.x = 0
            
            self.twist_command_publisher.publish(self.command_vel)
            if self.command_vel.linear.x==0:
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
    
    def swipe_right(self,X_right):
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
        self.command_vel.linear.x = VELOCITY_X

        i=0

        while not rospy.is_shutdown():
            
            if self.move_group.get_current_pose().pose.position.x < X_right:
                self.command_vel.linear.x = VELOCITY_X
            else:
                self.command_vel.linear.x = 0
            
            self.twist_command_publisher.publish(self.command_vel)
            if self.command_vel.linear.x==0:
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
        # rospy.loginfo("Waypoints : %s", waypoints)

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

    def swipe_callback(self, goal:SwipeGoal):
        rospy.loginfo("Received pick and place goal")
        stain_pose = goal.stain_pose
        x_min_y_min = goal.x_min_y_min
        x_max_y_max = goal.x_max_y_max
        
        success = self.swipe(stain_pose,x_min_y_min,x_max_y_max)
        
        # set goal to success
        result = SwipeResult()
        result.result = success

        if success:
            self.swipe_server.set_succeeded(result)
        else:
            self.swipe_server.set_aborted(result)


    def swipe(self,stain_pose,x_min_y_min,x_max_y_max):
        rospy.loginfo("Swipe started with stain_pose : %s", stain_pose)
        rospy.loginfo("bounding box (x1y1 -> x2y2) : %s %s", x_min_y_min, x_max_y_max)

        x_max_y_max_ref = copy.deepcopy(x_max_y_max)
        x_min_y_min_ref = copy.deepcopy(x_min_y_min)

        # check if location is to right or left and getting and start and end poses
        start_swipe = Pose()
        end_swipe = Pose()
        if stain_pose.pose.position.x > 0: # the dust is to the right, swipe right
            start_swipe = x_min_y_min.pose
            start_swipe.position.z = PRESWIPE_HEIGHT
            start_swipe.orientation = stain_pose.pose.orientation
            end_swipe = x_max_y_max.pose
            end_swipe.position.y = x_min_y_min.pose.position.y
            end_swipe.position.z = PRESWIPE_HEIGHT
            end_swipe.orientation = stain_pose.pose.orientation
            swipe_velocity = VELOCITY_X
            print("the object is to the right")
            print("start pose ; ",start_swipe)
            print("stain_pose ; ",stain_pose.pose)
            print("end_swipe : ",end_swipe)
            print("Velocity : ", swipe_velocity)

        if stain_pose.pose.position.x < 0: # the dust is to the left, swipe left
            start_swipe = x_max_y_max.pose
            start_swipe.position.y = x_min_y_min.pose.position.y
            start_swipe.position.z = PRESWIPE_HEIGHT
            start_swipe.orientation = stain_pose.pose.orientation
            end_swipe = x_min_y_min.pose
            end_swipe.position.z = PRESWIPE_HEIGHT
            end_swipe.orientation = stain_pose.pose.orientation
            swipe_velocity = -VELOCITY_X
            print("the object is to the left")
            print("start swipe ; ",start_swipe)
            print("stain_pose ; ",stain_pose.pose)
            print("end_swipe : ",end_swipe)
            print("Velocity : ", swipe_velocity)
        
        waypoints = []
        waypoints.append(copy.deepcopy(self.move_group.get_current_pose().pose))
        waypoints.append(copy.deepcopy(start_swipe))
        self.execute_waypoints(waypoints)

        # Start swiping
        print("Target Y : ",x_max_y_max.pose.position.y)
        while self.move_group.get_current_pose().pose.position.y < x_max_y_max_ref.pose.position.y:
            # go to start swipe
            print("shwiping")
            print("Current y : ",self.move_group.get_current_pose().pose.position.y)
            print("Target y : ",x_max_y_max.pose.position.y)
            print("Target ` y : ",x_max_y_max_ref.pose.position.y)
            waypoints = []
            waypoints.append(copy.deepcopy((self.move_group.get_current_pose()).pose))
            waypoints.append(copy.deepcopy(start_swipe))
            self.execute_waypoints(waypoints=waypoints)
            # touch at start swipe
            # self.touch_ft_feedback()

            # swipe now, left or right based on stain_pose.position
            # if stain_pose.pose.position.x < 0:
            #     self.swipe_left(end_swipe.position.x)
            # if stain_pose.pose.position.x > 0:
            #     self.swipe_right(end_swipe.position.x)

            waypoints = []
            waypoints.append(copy.deepcopy((self.move_group.get_current_pose()).pose))
            waypoints.append(copy.deepcopy(end_swipe))
            start_swipe.position.y += Y_STEP
            end_swipe.position.y += Y_STEP
            waypoints.append(copy.deepcopy(start_swipe))
            print("Current y : ",self.move_group.get_current_pose().pose.position.y)
            print("Target y : ",x_max_y_max.pose.position.y)
            print("Target ` y : ",x_max_y_max_ref.pose.position.y)
            self.execute_waypoints(waypoints=waypoints)
        return True

        
if __name__  == "__main__":
    rospy.init_node("right_swipe_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)

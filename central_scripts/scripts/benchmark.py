import rospy
import copy
import cv_bridge
import cv2
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal
from create_2025_mp_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal
import actionlib
from openai import OpenAI
import numpy as np
from ur_msgs.srv import SetIO
import moveit_commander
import moveit_msgs.msg
from math import pi, tau, dist, fabs, cos
from tf.transformations import quaternion_from_euler, quaternion_multiply
from moveit_commander.conversions import pose_to_list
import sys


## for benchmarking the calibration, use basic opencv based perception to detect red circle
## configurable arm responsibilities : which to use to look and which to use to move to the circle

### Configure arm responsibilities
look_arm = "right"
go_arm = "left"

### Configure goto height ###
goto_height =  0.2

#### Define go orientation #######
ORIENTATION_POSE = PoseStamped()
ORIENTATION_POSE.pose.orientation.x= -0.7084016817823435
ORIENTATION_POSE.pose.orientation.y= 0.7057186070566935
ORIENTATION_POSE.pose.orientation.z= 0.007889737191896513
ORIENTATION_POSE.pose.orientation.w= 0.008127542614487311


class CentralClient:
    def __init__(self) -> None:

        rospy.loginfo("Waiting for servers")
        self.right_pick_place_client = actionlib.SimpleActionClient("right_pick_place", PickPlaceAction)
        self.right_move_preaction_client = actionlib.SimpleActionClient("right_move_preaction", MovePreactionAction)
        self.right_move_look_client = actionlib.SimpleActionClient("right_move_look", MovePreactionAction)
        self.right_move_rest_client = actionlib.SimpleActionClient("right_move_rest", MovePreactionAction)
        self.left_pick_place_client = actionlib.SimpleActionClient("left_pick_place", PickPlaceAction)
        self.left_move_preaction_client = actionlib.SimpleActionClient("left_move_preaction", MovePreactionAction)
        self.left_move_look_client = actionlib.SimpleActionClient("left_move_look", MovePreactionAction)
        self.left_move_rest_client = actionlib.SimpleActionClient("left_move_rest", MovePreactionAction)
        rospy.sleep(0.1)
        self.right_pick_place_client.wait_for_server()
        self.right_move_preaction_client.wait_for_server()
        self.left_pick_place_client.wait_for_server()
        self.left_move_preaction_client.wait_for_server()
        self.right_move_look_client.wait_for_server()
        self.right_move_rest_client.wait_for_server()
        self.left_move_look_client.wait_for_server()
        self.left_move_rest_client.wait_for_server()
        
        # moveit initialization
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("Initializing motion planner")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.right_arm_group_name = "right_arm"
        self.right_arm_move_group = moveit_commander.MoveGroupCommander(self.right_arm_group_name)

        # get the planning frame
        right_arm_planning_frame = self.right_arm_move_group.get_planning_frame()
        print("Right Planning frame : %s" %right_arm_planning_frame)

        self.left_arm_group_name = "left_arm"
        self.left_arm_move_group = moveit_commander.MoveGroupCommander(self.left_arm_group_name)

        # get the planning frame
        left_arm_planning_frame = self.right_arm_move_group.get_planning_frame()
        print("Left Planning frame : %s" %left_arm_planning_frame)

        # get the end effector link
        right_arm_eef_link = self.right_arm_move_group.get_end_effector_link()
        print("Right End effector link : %s" % right_arm_eef_link)

        # get the end effector link
        left_arm_eef_link = self.left_arm_move_group.get_end_effector_link()
        print("Left End effector link : %s" % left_arm_eef_link)

        # get all the group names in the robot
        group_names = self.robot.get_group_names()
        print("All planning groups : %s" %group_names)

        # print the entire state of the robot
        print("Robot state ")
        print(self.robot.get_current_state())


        # Subscribers to cameras
        self.depth_image_topic = "/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/aligned_depth_to_color/camera_info"
        self.color_image_topic = "/camera/color/image_raw"
        
        # Detected circle
        self.red_fiducial_detected = None

        self.depth_image_sub = rospy.Subscriber(
                                                "/camera/aligned_depth_to_color/image_raw", 
                                                Image, 
                                                self.depth_image_callback
                                                )
        self.camera_info_sub = rospy.Subscriber(
                                                "/camera/aligned_depth_to_color/camera_info", 
                                                CameraInfo, 
                                                self.camera_info_callback
                                                )
        self.color_image_sub = rospy.Subscriber(
                                                "/camera/color/image_raw",
                                                Image,
                                                self.color_image_callback
                                                )
    
        self.get_red_fiducial_position_timer = rospy.Timer(period=0.05,callback=self.get_red_fiducial_location)
        self.publish_red_fiducial_position_timer = rospy.Timer(period=0.05,callback=self.publish_stream)
        self.stream_pub = rospy.Publisher("/detected_red_fiducial", PoseStamped, queue_size=10)

        rospy.loginfo("All servers are connected")

    def color_image_callback(self, msg: Image):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pass

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pass
    
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)
        pass

    def execute_waypoints(self, waypoints, move_group):
        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints : %s", waypoints)

        # plan a cartesian path
        try : 
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
        except Exception as e:
            print(e)
            return False
        
        rospy.loginfo(f"Fraction : {fraction}")

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing prepick")
        try : 
            move_group.execute(plan, wait=True)
            move_group.stop()
        except Exception as e:
            print(e)
            return False

    # execute all the actions in the action list right one by one here.
    def cartesian_goto_right(self, destination:PoseStamped):
        destination_pose = destination.pose
        destination_pose.orientation = ORIENTATION_POSE.pose.orientation
        waypoints = []
        waypoint1 = destination_pose
        waypoint1.position.z = goto_height
        waypoints.append(copy.deepcopy(waypoint1))
        waypoint2 = destination_pose
        waypoints.append(copy.deepcopy(waypoint2))
        self.execute_waypoints(waypoints=waypoints,move_group=self.right_arm_move_group)
       
        input("Press Enter to continue")
        waypoints = []
        waypoints.append(copy.deepcopy(waypoint1))
        self.execute_waypoints(waypoints=waypoints,move_group=self.right_arm_move_group)


    # execute all the actions in the action list right one by one here.
    def cartesian_goto_left(self, destination:PoseStamped):
        destination_pose = destination.pose
        destination_pose.orientation = ORIENTATION_POSE.pose.orientation
        waypoints = []
        waypoint1 = destination_pose
        waypoint1.position.z = goto_height
        waypoints.append(copy.deepcopy(waypoint1))
        waypoint2 = destination_pose
        waypoints.append(copy.deepcopy(waypoint2))
        self.execute_waypoints(waypoints=waypoints,move_group=self.left_arm_move_group)
       
        input("Press Enter to continue")
        waypoints = []
        waypoints.append(copy.deepcopy(waypoint1))
        self.execute_waypoints(waypoints=waypoints,move_group=self.left_arm_move_group)

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        try:
            # Initialize the tf2 transform buffer and listener
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)

            # Wait for the transform to become available
            tf_buffer.can_transform(target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(3.0))

            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                pose,
                tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rospy.Time(0))
            )
            return transformed_pose
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup error: {e}")
        except tf2_ros.ConnectivityException as e:
            rospy.logerr(f"Transform connectivity error: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation error: {e}")
        return None
    
    def get_3d_position(self, x, y):
        if self.depth_image is None and self.camera_info is None:
            return  # Wait until depth image is received
        depth = (self.depth_image[y, x]/1000)  # Convert to meters
        if np.isnan(depth) or depth == 0:
            rospy.logwarn("Invalid depth at pixel ({}, {})".format(x, y))
            self.recursion +=1
            if self.recursion <= 10:
                self.get_3d_position(x,y)
            else: 
                self.recursion = 0
                return None
            
        # Project the 2D pixel to 3D point in the camera frame
        point_3d = self.camera_model.projectPixelTo3dRay((x, y))
        print("X multiplier : ",point_3d[0])
        print("Y multiplier : ",point_3d[1])
        point_3d = np.array(point_3d) * depth  # Scale the ray by the depth
        pose = PoseStamped()
        pose.header.frame_id = self.camera_model.tf_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point_3d[0]
        pose.pose.position.y = point_3d[1]
        pose.pose.position.z = point_3d[2]
        pose.pose.orientation.w = 1.0

        # transform the pose to the world frame and encapsulate it in a try except block
        try:
            transformed_pose = self.transform_pose(pose, "world")
            if transformed_pose is None:
                rospy.logwarn("Failed to transform the pose to the world frame")
                return
            return transformed_pose
        except Exception as e:
            rospy.logerr(f"Error transforming pose: {e}")
            return None
    
    def get_red_fiducial_location(self):
        # Load the image
        rospy.wait_for_message(self.color_image_topic,Image)
        frame = self.color_image
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for red color (filled red color detection)
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 150])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the HSV image to get only red colors
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Reduce noise
        mask = cv2.medianBlur(mask, 5)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Approximate a circle using minimum enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)

        # get deprojected coordinates 
        red_fiducial_position = self.get_3d_position(x,y)
        
        if red_fiducial_position is not None:
            self.red_fiducial_detected = red_fiducial_position
            return red_fiducial_position


    def publish_stream(self):
        if self.red_fiducial_detected is not None :            
            self.stream_pub.publish(self.red_fiducial_detected)


if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.2)

    time = rospy.Time.now()

    # send both the arms to rest
    move_preaction_goal = MovePreactionActionGoal()
    central_client.left_move_rest_client.send_goal(move_preaction_goal)
    central_client.left_move_rest_client.wait_for_result()
    move_preaction_goal = MovePreactionActionGoal()
    central_client.right_move_rest_client.send_goal(move_preaction_goal)
    central_client.right_move_rest_client.wait_for_result()

    i = 0
    while True :     
        input(f"Press Enter to start iteration {i}")
        i+=1
        
        # move the required arm to look state
        if look_arm == "right":
            central_client.right_move_look_client.send_goal(move_preaction_goal)
            central_client.right_move_look_client.wait_for_result()
        if look_arm == "left":
            central_client.left_move_look_client.send_goal(move_preaction_goal)
            central_client.left_move_look_client.wait_for_result()

        rospy.sleep(0.2)
        rospy.loginfo("Get the marker location")
        red_fiducial_location = central_client.get_red_fiducial_location()
        # Add offsets here

        rospy.loginfo("Detected Red fiducial : %s" %red_fiducial_location)
        rospy.loginfo("Now moving the %s arm to go to detected fiducial" %go_arm)

        if go_arm == "right":
            central_client.left_move_rest_client.send_goal(move_preaction_goal)
            central_client.left_move_rest_client.wait_for_result()
        if go_arm == "left":
            central_client.right_move_rest_client.send_goal(move_preaction_goal)
            central_client.right_move_rest_client.wait_for_result()

        if go_arm == "right":
            central_client.cartesian_goto_right(red_fiducial_location)
        if go_arm == "left":
            central_client.cartesian_goto_left(red_fiducial_location)

        if look_arm != go_arm :
            if go_arm == "right":
                central_client.right_move_rest_client.send_goal(move_preaction_goal)
                central_client.right_move_rest_client.wait_for_result()
            if go_arm == "left":
                central_client.left_move_rest_client.send_goal(move_preaction_goal)
                central_client.left_move_rest_client.wait_for_result()
        
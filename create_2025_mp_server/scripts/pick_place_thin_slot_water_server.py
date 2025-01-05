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
# move up or down, set gripper value, time before activating gripper and moving, handover pose, pre handover pose

### CREATE X Y Z LITERALS FOR PADDING between end effector ###
PADDING_X = 0.0
PADDING_Y = 0.0
PADDING_Z = 0.0
#########################################

#### handover pose defn ####
handover_pose_left = PoseStamped()
handover_pose_left.header.frame_id = "world"
handover_pose_left.pose.position.x = 0.5
handover_pose_left.pose.position.y = 0.0
handover_pose_left.pose.position.z = 0.5
handover_pose_left.pose.orientation.x = 0.0
handover_pose_left.pose.orientation.y = 0.0
handover_pose_left.pose.orientation.z = 0.0
handover_pose_left.pose.orientation.w = 1.0

handover_pose_right = PoseStamped()
handover_pose_right.header.frame_id = "world"
handover_pose_right.pose.position.x = 0.5
handover_pose_right.pose.position.y = 0.0
handover_pose_right.pose.position.z = 0.5
handover_pose_right.pose.orientation.x = 0.0
handover_pose_right.pose.orientation.y = 0.0
handover_pose_right.pose.orientation.z = 0.0
handover_pose_right.pose.orientation.w = 1.0

##############################################



class Motion_planner:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.loginfo("Initializing motion planner")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name_right = "right_arm"
        self.move_group_right = moveit_commander.MoveGroupCommander(self.group_name_right)
        self.group_name_left = "left_arm"
        self.move_group_left = moveit_commander.MoveGroupCommander(self.group_name_left)


        # get the planning frame
        print("##################################################")
        planning_frame = self.move_group_right.get_planning_frame()
        print("Planning frame : %s" %planning_frame)

        # get the end effector link
        # im guessing that this configuration for the move_group id is set in the 
        # moveit configuration file
        eef_link = self.move_group_right.get_end_effector_link()
        print("End effector link : %s" % eef_link)

        # get all the group names in the robot
        group_names = self.robot.get_group_names()
        print("All planning groups : %s" %group_names)

        # print the entire state of the robot
        print("Robot state ")
        print(self.robot.get_current_state())
        
        print("##################################################")
        planning_frame = self.move_group_left.get_planning_frame()
        print("Planning frame : %s" %planning_frame)

        # get the end effector link
        # im guessing that this configuration for the move_group id is set in the 
        # moveit configuration file
        eef_link = self.move_group_left.get_end_effector_link()
        print("End effector link : %s" % eef_link)

        # get all the group names in the robot
        group_names = self.robot.get_group_names()
        print("All planning groups : %s" %group_names)

        # print the entire state of the robot
        print("Robot state ")
        print(self.robot.get_current_state())

        print("##################################################")

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.waypoints = []

        # create action server for pick and place
        self.pick_place_server = actionlib.SimpleActionServer(
            "pick_place_thin_slot", PickPlaceAction, self.pick_place_callback, auto_start=False
        )

        
        # create a service client for /left/ur_hardware_interface/set_io
        self.left_set_io_client = rospy.ServiceProxy("/left/ur_hardware_interface/set_io", SetIO)
        self.left_set_io_client.wait_for_service()
        self.right_set_io_client = rospy.ServiceProxy("/right/ur_hardware_interface/set_io", SetIO)
        self.right_set_io_client.wait_for_service()
        
        self.pick_place_server.start()



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

        # plan a cartesian path for right to pick, prepick -> pick
        waypoints = []
        initial_pose = self.move_group_right.get_current_pose().pose
        prepick = Pose()
        prepick = start.pose
        prepick.position.z += 0.2# move up 20 cm
        waypoints.append(copy.deepcopy(initial_pose))
        waypoints.append(copy.deepcopy(prepick))
        
        pick = copy.deepcopy(prepick)
        pick.position.z -= 0.2 # move down 20 cm
        waypoints.append(copy.deepcopy(pick))

        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints for right arm current to pick: %s", waypoints)
        # plan a cartesian path for right
        try : 
            (plan, fraction) = self.move_group_right.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
        except Exception as e:
            print(e)
            return False

        # display the plan for right
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan for right
        rospy.loginfo("Executing prepick -> pick plan")
        try : 
            self.move_group_right.execute(plan, wait=True)
            self.move_group_right.stop()
        except Exception as e:
            print(e)
            return False


        rospy.sleep(1)

        # activate the gripper here for right
        rospy.loginfo("Activating gripper")
        self.right_set_io_client(1,12,1)

        rospy.sleep(1)


        # plan cartesian path from prepick to handover pose for right
        waypoints = []
        current_pose = self.move_group_right.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(prepick))
        up_jerk = copy.deepcopy(prepick)
        up_jerk.position.z += 0.015
        low_jerk = copy.deepcopy(up_jerk)
        low_jerk.position.z -= 0.015
        waypoints.append(copy.deepcopy(up_jerk))
        waypoints.append(copy.deepcopy(low_jerk))
        waypoints.append(copy.deepcopy(up_jerk))
        waypoints.append(copy.deepcopy(low_jerk))
        waypoints.append(copy.deepcopy(up_jerk))
        waypoints.append(copy.deepcopy(low_jerk))
        waypoints.append(copy.deepcopy(up_jerk))
        waypoints.append(copy.deepcopy(low_jerk))
        waypoints.append(copy.deepcopy(up_jerk))
        waypoints.append(copy.deepcopy(low_jerk))
        waypoints.append(copy.deepcopy(handover_pose_right.pose))

        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints for right arm, pick to handover pose: %s", waypoints)

        # plan a cartesian path for right
        try :
            (plan, fraction) = self.move_group_right.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
        except Exception as e:
            print(e)
            return False
        
        # display the plan for right
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan for right
        rospy.loginfo("Executing prepick -> handover plan")
        try :
            self.move_group_right.execute(plan, wait=True)
            self.move_group_right.stop()
        except Exception as e:
            print(e)
            return False
        
        # plan a cartesian path for left from current to pre handover to handover to prehandover
        waypoints = []
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        pre_handover_left = Pose()
        pre_handover_left = handover_pose_left.pose
        
        ###################### pre handover configuration ######################
        pre_handover_left.position.x -= 0.3
        pre_handover_left.position.y -= 0.3

        waypoints.append(copy.deepcopy(pre_handover_left))
        waypoints.append(copy.deepcopy(handover_pose_left.pose))

        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints for left arm, current to handover: %s", waypoints)

        # plan a cartesian path for left
        try :
            (plan, fraction) = self.move_group_left.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
        except Exception as e:
            print(e)
            return False
        
        # display the plan for left
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan for left
        rospy.loginfo("Executing current -> handover plan for left")
        try :
            self.move_group_left.execute(plan, wait=True)
            self.move_group_left.stop()
        except Exception as e:
            print(e)
            return False
        

        # activate  the left gripper here
        rospy.loginfo("Activating left gripper")
        self.left_set_io_client(1,12,1)
        rospy.sleep(1)
        
        # deactivate the right gripper here
        rospy.loginfo("Deactivating right gripper")
        self.right_set_io_client(1,12,0)
        rospy.sleep(1)

        # now the handover is done
        # plan a cartesian path for left from handover to preplace
        waypoints = []
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(handover_pose_left.pose))
        preplace = Pose()
        preplace = end.pose
        # move up 20 cm
        preplace.position.z += 0.20
        waypoints.append(copy.deepcopy(preplace))
        place = copy.deepcopy(preplace)
        # move down 20 cm
        place.position.z -= 0.20
        waypoints.append(copy.deepcopy(place))

        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints for left arm, handover to place: %s", waypoints)

        # plan a cartesian path for left
        try :
            (plan, fraction) = self.move_group_left.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.005,  # eef_step
            )
        except Exception as e:
            print(e)
            return False

        # display the plan for left
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan for left
        rospy.loginfo("Executing handover -> place plan for left")
        try :
            self.move_group_left.execute(plan, wait=True)
            self.move_group_left.stop()
        except Exception as e:
            print(e)
            return False

        rospy.sleep(1)

        # deactivate the left gripper here
        rospy.loginfo("Deactivating left gripper")
        self.left_set_io_client(1,12,0)
        rospy.sleep(1)

        return True
        
if __name__  == "__main__":
    rospy.init_node("right_pick_place_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        



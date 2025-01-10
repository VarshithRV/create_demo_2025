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

#### right handover defenition #####
from geometry_msgs.msg import Pose

# Post pick pose
right_post_pick_pose = Pose()
right_post_pick_pose.position.x = -0.1434408962398437
right_post_pick_pose.position.y = 0.04839345698321429
right_post_pick_pose.position.z = 0.24155259699598822
right_post_pick_pose.orientation.x = 0.05226904384319071
right_post_pick_pose.orientation.y = -0.9981268826449426
right_post_pick_pose.orientation.z = -0.02830494394839869
right_post_pick_pose.orientation.w = 0.014474230386632456

# Pre-transfer pose 1
right_pre_transfer_pose_1 = Pose()
right_pre_transfer_pose_1.position.x = 0.08063571531701268
right_pre_transfer_pose_1.position.y = -0.09343200361710771
right_pre_transfer_pose_1.position.z = 0.24403899095661546
right_pre_transfer_pose_1.orientation.x = -0.04004417828833372
right_pre_transfer_pose_1.orientation.y = 0.9990158656635603
right_pre_transfer_pose_1.orientation.z = 0.01905227149347686
right_pre_transfer_pose_1.orientation.w = 0.0008802775915563199

# Pre-transfer pose 2
right_pre_transfer_pose_2 = Pose()
right_pre_transfer_pose_2.position.x = 0.2993811777529541
right_pre_transfer_pose_2.position.y = -0.04530481612116087
right_pre_transfer_pose_2.position.z = 0.26711419609996645
right_pre_transfer_pose_2.orientation.x = -0.1759530688930912
right_pre_transfer_pose_2.orientation.y = -0.9840877695361997
right_pre_transfer_pose_2.orientation.z = -0.023362990654236422
right_pre_transfer_pose_2.orientation.w = 0.008120964478473013

# Transfer pose
right_transfer_pose = Pose()
right_transfer_pose.position.x = 0.2993811777529541
right_transfer_pose.position.y = -0.04530481612116087
right_transfer_pose.position.z = 0.26711419609996645
right_transfer_pose.orientation.x = -0.1759530688930912
right_transfer_pose.orientation.y = -0.9840877695361997
right_transfer_pose.orientation.z = -0.023362990654236422
right_transfer_pose.orientation.w = 0.008120964478473013

# Post transfer pose
right_post_transfer_pose = Pose()
right_post_transfer_pose.position.x = 0.3023839939075332
right_post_transfer_pose.position.y = -0.08948719347340107
right_post_transfer_pose.position.z = 0.28592881103513035
right_post_transfer_pose.orientation.x = -0.3333775580700785
right_post_transfer_pose.orientation.y = 0.23220504626341534
right_post_transfer_pose.orientation.z = 0.527831843049173
right_post_transfer_pose.orientation.w = 0.7458778490666884

####################################



#### left handover pose defn ####
left_pre_transfer_pose_1 = Pose() 
left_pre_transfer_pose_1.position.x = 0.33128672377620716
left_pre_transfer_pose_1.position.y = 0.27895466285275816
left_pre_transfer_pose_1.position.z = 0.6245202023267862
left_pre_transfer_pose_1.orientation.x = 0.8959475579214162
left_pre_transfer_pose_1.orientation.y = 0.441090108527203
left_pre_transfer_pose_1.orientation.z = 0.011897590405931972
left_pre_transfer_pose_1.orientation.w = 0.05075368909393231

left_pre_transfer_pose_2 = Pose()
left_pre_transfer_pose_2.position.x = 0.3338362855877785
left_pre_transfer_pose_2.position.y = 0.2869326187384163
left_pre_transfer_pose_2.position.z = 0.5860401783032727
left_pre_transfer_pose_2.orientation.x = 0.8113622750259434
left_pre_transfer_pose_2.orientation.y = 0.46277799156838556
left_pre_transfer_pose_2.orientation.z = -0.21350445330198808
left_pre_transfer_pose_2.orientation.w = 0.28625799133801666

left_transfer_pose = Pose()
left_transfer_pose.position.x = 0.3338362855877785
left_transfer_pose.position.y = 0.2869326187384163
left_transfer_pose.position.z = 0.5860401783032727
left_transfer_pose.orientation.x = 0.8113622750259434
left_transfer_pose.orientation.y = 0.46277799156838556
left_transfer_pose.orientation.z = -0.21350445330198808
left_transfer_pose.orientation.w = 0.28625799133801666

left_post_transfer_pose_1 = Pose()
left_post_transfer_pose_1.position.x = 0.310876911801094
left_post_transfer_pose_1.position.y = 0.21680922659155777
left_post_transfer_pose_1.position.z = 0.5180625803281337
left_post_transfer_pose_1.orientation.x= 0.8289269482775882
left_post_transfer_pose_1.orientation.y= 0.39465910718018804
left_post_transfer_pose_1.orientation.z =-0.20015493061560463
left_post_transfer_pose_1.orientation.w= 0.3421436939199728

left_post_transfer_pose_2 = Pose()
left_post_transfer_pose_2.position.x = 0.33128386303159024
left_post_transfer_pose_2.position.y = 0.27895986996191846
left_post_transfer_pose_2.position.z = 0.6244985867969725
left_post_transfer_pose_2.orientation.x= 0.8959524850199384
left_post_transfer_pose_2.orientation.y= 0.44108456086259523
left_post_transfer_pose_2.orientation.z= 0.011900956685154198
left_post_transfer_pose_2.orientation.w= 0.050714120176009854

left_post_transfer_pose_3 = Pose()
left_post_transfer_pose_3.position.x = 0.3317781031241287
left_post_transfer_pose_3.position.y = 0.28140795598686613
left_post_transfer_pose_3.position.z = 0.34983345155523854
left_post_transfer_pose_3.orientation.x = 0.896911293745636
left_post_transfer_pose_3.orientation.y = 0.4391405340103312
left_post_transfer_pose_3.orientation.z = 0.011292044036456237
left_post_transfer_pose_3.orientation.w = 0.050776099516695004

left_post_place_pose = Pose()
left_post_place_pose.position.x = -0.20655276016359186
left_post_place_pose.position.y = 0.23309133048431538
left_post_place_pose.position.z = 0.3754173005227305
left_post_place_pose.orientation.x = 0.9023133923705515
left_post_place_pose.orientation.y =0.4305577799600779
left_post_place_pose.orientation.z =0.02041269968522073
left_post_place_pose.orientation.w =0.005819085508683705 
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
            "pick_place_handover", PickPlaceAction, self.pick_place_callback, auto_start=False
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

    def execute_waypoint_right(self,waypoints : list):
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
        
    def execute_waypoint_left(self,waypoints : list):
        rospy.loginfo("#################################")
        rospy.loginfo("Waypoints for right arm current to pick: %s", waypoints)
        # plan a cartesian path for right
        try : 
            (plan, fraction) = self.move_group_left.compute_cartesian_path(
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
            self.move_group_left.execute(plan, wait=True)
            self.move_group_left.stop()
        except Exception as e:
            print(e)
            return False

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

        pick_place_height = 0.4271641690575525


        # plan a cartesian path for right to pick, prepick -> pick
        waypoints = []
        initial_pose = self.move_group_right.get_current_pose().pose
        prepick = Pose()
        prepick = start.pose
        prepick.position.z = pick_place_height
        waypoints.append(copy.deepcopy(initial_pose))
        waypoints.append(copy.deepcopy(prepick))
        self.execute_waypoint_right(waypoints)

        waypoints = []
        pick = copy.deepcopy(start.pose)
        current = self.move_group_right.get_current_pose().pose
        waypoints.append(copy.deepcopy(current))
        waypoints.append(copy.deepcopy(pick))
        
        self.execute_waypoint_right(waypoints)

        rospy.sleep(1)

        # activate the gripper here for right
        rospy.loginfo("Activating gripper")
        self.right_set_io_client(1,12,1)

        rospy.sleep(1)

        # pickup
        waypoints = []
        current_pose = self.move_group_right.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(prepick))
        self.execute_waypoint_right(waypoints)
        rospy.sleep(0.2)

        # starting the right pre transfer sequence
        # right pre transfer 1
        waypoints = [] 
        current_pose = self.move_group_right.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(right_pre_transfer_pose_1))
        self.execute_waypoint_right(waypoints)
        rospy.sleep(0.2)

        # right pre transfer 2
        waypoints = [] 
        current_pose = self.move_group_right.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(right_pre_transfer_pose_2))
        self.execute_waypoint_right(waypoints)
        rospy.sleep(0.2)
        
        # right transfer
        waypoints = [] 
        current_pose = self.move_group_right.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(right_transfer_pose))
        self.execute_waypoint_right(waypoints)
        rospy.sleep(0.2)


        # start left pre transfer
        # left pre transfer 1
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(left_pre_transfer_pose_1))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(0.2)

        # left pre transfer 2
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(left_pre_transfer_pose_2))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(0.2)

        # left transfer
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(left_transfer_pose))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(1)

        # activate  the left gripper here
        rospy.loginfo("Activating left gripper")
        self.left_set_io_client(1,12,1)
        rospy.sleep(1)
        
        # deactivate the right gripper here
        rospy.loginfo("Deactivating right gripper")
        self.right_set_io_client(1,12,0)
        rospy.sleep(1)

        # right post transfer
        waypoints = [] 
        current_pose = self.move_group_right.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(right_post_transfer_pose))
        self.execute_waypoint_right(waypoints)
        rospy.sleep(0.2)

        # left post transfer and place
        # left post transfer 1
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(left_post_transfer_pose_1))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(0.2)

        # left post transfer 2
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(left_post_transfer_pose_2))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(0.2)

        # left post transfer 3
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(left_post_transfer_pose_3))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(0.2)

        # left preplace
        preplace = Pose()
        preplace = copy.deepcopy(end.pose)
        preplace.position.z = pick_place_height
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(preplace))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(0.2)

        # left place
        waypoints = [] 
        current_pose = self.move_group_left.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))
        waypoints.append(copy.deepcopy(end.pose))
        self.execute_waypoint_left(waypoints)
        rospy.sleep(0.2)


        
if __name__  == "__main__":
    rospy.init_node("right_pick_place_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
        



import rospy
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse, GetObjectLocationsRequest
import cv_bridge
import cv2
from PIL import Image
import io
import base64
import json
import requests
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from std_msgs.msg import String
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult, SwipeAction, SwipeGoal, SwipeResult
from create_2025_mp_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult, PoseAction, PoseGoal, PoseResult
import actionlib
from openai import OpenAI
import numpy as np
from ur_msgs.srv import SetIO
import sys

#### Define drope pose #########
DROP_POSE = PoseStamped()
DROP_POSE.pose.position.x= 0.1
DROP_POSE.pose.position.y= 0.35
DROP_POSE.pose.position.z= 0.05
DROP_POSE.pose.orientation.x= -0.7084016817823435
DROP_POSE.pose.orientation.y= 0.7057186070566935
DROP_POSE.pose.orientation.z= 0.007889737191896513
DROP_POSE.pose.orientation.w= 0.008127542614487311
#################################

#### Define pick place orientation #######
RIGHT_ORIENTATION_POSE = PoseStamped()
RIGHT_ORIENTATION_POSE.pose.orientation.x = 0.9168428669078561
RIGHT_ORIENTATION_POSE.pose.orientation.y = 0.3981116253989925
RIGHT_ORIENTATION_POSE.pose.orientation.z = 0.004269784548492005
RIGHT_ORIENTATION_POSE.pose.orientation.w = 0.029800336613282526

LEFT_ORIENTATION_POSE = PoseStamped()
LEFT_ORIENTATION_POSE.pose.orientation.x= -0.6955566827400104
LEFT_ORIENTATION_POSE.pose.orientation.y= -0.7184260972597232
LEFT_ORIENTATION_POSE.pose.orientation.z= -0.006965116877090745
LEFT_ORIENTATION_POSE.pose.orientation.w= 0.004041165520492587

TEXT_PROMPT = ".red_ball.blue_dirt.red_dirt."

class CentralClient:
    def __init__(self) -> None:
        self.get_object_locations_service = rospy.ServiceProxy(
            "left_get_object_locations",
            GetObjectLocations
        )

        rospy.loginfo("Waiting for servers")
        self.right_pick_place_client = actionlib.SimpleActionClient("right_pick_place", PickPlaceAction)
        self.right_move_preaction_client = actionlib.SimpleActionClient("right_move_preaction", MovePreactionAction)
        self.right_move_look_client = actionlib.SimpleActionClient("right_move_look", MovePreactionAction)
        self.right_move_rest_client = actionlib.SimpleActionClient("right_move_rest", MovePreactionAction)
        self.left_pick_place_client = actionlib.SimpleActionClient("left_pick_place", PickPlaceAction)
        self.left_move_preaction_client = actionlib.SimpleActionClient("left_move_preaction", MovePreactionAction)
        self.left_move_look_client = actionlib.SimpleActionClient("left_move_look", MovePreactionAction)
        self.left_move_rest_client = actionlib.SimpleActionClient("left_move_rest", MovePreactionAction)
        self.right_place_client = actionlib.SimpleActionClient("right_place", PoseAction)
        self.right_pick_client = actionlib.SimpleActionClient("right_pick", PoseAction)
        self.right_swipe_client = actionlib.SimpleActionClient("right_swipe", SwipeAction)
        rospy.sleep(0.1)
        self.right_pick_place_client.wait_for_server()
        self.right_move_preaction_client.wait_for_server()
        self.left_pick_place_client.wait_for_server()
        self.left_move_preaction_client.wait_for_server()
        self.right_move_look_client.wait_for_server()
        self.right_move_rest_client.wait_for_server()
        self.left_move_look_client.wait_for_server()
        self.left_move_rest_client.wait_for_server()
        self.right_place_client.wait_for_server()
        self.right_pick_client.wait_for_server()
        self.right_swipe_client.wait_for_server()
        rospy.loginfo("All servers are connected")

    def get_object_locations(self):
        try:
            request = GetObjectLocationsRequest()
            request.prompt.data = TEXT_PROMPT
            rospy.loginfo("Request for perception = {request}")
            response = self.get_object_locations_service(request)
            return response
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
  
    def llm(self, prompt, object_detections, annotated_image):

        # process image into the prompt as well
        def encode_image(image):
            buffer = cv2.imencode('.jpg', image)[1]
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            return image_base64

        print("In llm call")
        print("prompt : ",prompt)
        # print("object detections : ",object_detections)
        base64_annotated_image = encode_image(annotated_image)

        dict_obj_list = []
        for object in object_detections:
            dict_obj = {"id" : object.id, "label": object.Class}
            dict_obj_list.append(dict_obj)

        json_detections = json.dumps(dict_obj_list, indent=2)
<<<<<<< HEAD
        preamble = "You are a robot controller, you need to write a sequence of actions. In the image, there is a red_ball, the red ball is attached to a brush that can be used to clean, there are also a few dirt patches, you job is to choose the right dirt patch to clean, to clean the patch with object_id_1 and object_id_2, you need to output the following : \{'clean':[object_id_1,object_id_2]\}, make sure the output format is adhered, do not include anything other than the output."
=======
        preamble = "You are a robot controller, you need to write a sequence of actions. In the image, there are different geometric shapes. You can only execute two types of actions: \"pick_using_left_arm\", \"pick_using_right_arm\", chose the appropriate action for the object depending on the prompt. The output needs to be in the following formats : {\"pick_using_left_arm\":[<object_id1>,<object_id2>, ...],\"pick_using_right_arm\":[<object_id3>, <object_id4>, ... ]}, this output means that the objects_id 1,2,3,4 .... need to be picked up, object id 1,2 .... need to be picked up using left arm and object id 3, 4 .... need to be picked up using right arm, if its ambigous, pick using the left arm, for objects that are 3D and not planar, use the right arm if not specified. Make sure the output format is adhered, do not include any more description of the reasoning. Refer the image to see which objects are where"
>>>>>>> fbf42d480fd2b7481ec059cbfee39a9c905abc0c
        client = OpenAI()

        completion = client.chat.completions.create(
            model="gpt-4o", 
            messages=[
                {
                    "type" : "text",
                    "role": "system", 
                    "content": preamble
                },
                {
                    "type" : "text",
                    "role": "user", 
                    "content": f"User Prompt: {prompt}\nImage Annotations: {json_detections}"
                },
                {
                    "type": "image_url",
                    "role":"user",
                    "content" : "This is the image",
                    "image_url" : {"url": f"data:image/jpg;base64,{base64_annotated_image}"}
                }

            ],
            max_tokens=150,
            temperature=0
        )
        # rospy.loginfo(f"The output of llm : {completion.choices[0].message.content}")

        # Return the generated response
        pick_list = json.loads(completion.choices[0].message.content)
        for object in pick_list["clean"]:
            object = int(object)
        # for object in pick_list["pick_using_right_arm"]:
        #     object = int(object)

        rospy.loginfo(f"The llm returned with object list : {pick_list}")

        return pick_list
        

    # execute all the actions in the action list right one by one here.
    def execute_actions_right(self, action_list):
        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.right_move_rest_client.send_goal(move_preaction_goal)
        self.right_move_rest_client.wait_for_result()
        move_preaction_result = self.right_move_look_client.get_result()
        self.left_move_rest_client.send_goal(move_preaction_goal)
        self.left_move_rest_client.wait_for_result()
        move_preaction_result = self.left_move_rest_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Executing actions ...")
        
        for action in action_list:
            source = action["source_object_position"]
            destination = action["target_object_position"]
            prompt = String()
            prompt.data = action["label"]
            rospy.loginfo("Sending pick and place goal")
            pick_place_goal = PickPlaceGoal()
            pick_place_goal.source = source
            pick_place_goal.destination = destination
            pick_place_goal.prompt = prompt
            rospy.loginfo(f"Calling pick place with prompt : %s" %pick_place_goal.prompt)
            self.right_pick_place_client.send_goal(pick_place_goal)
            self.right_pick_place_client.wait_for_result()
            pick_place_result = self.right_pick_place_client.get_result()
            print("Pick and place result : ", pick_place_result.result)
            rospy.sleep(1)

        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.right_move_rest_client.send_goal(move_preaction_goal)
        self.right_move_rest_client.wait_for_result()
        move_preaction_result = self.right_move_rest_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Done")

    # execute all the actions in the action list right one by one here.
    def execute_actions_left(self, action_list):
        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.right_move_rest_client.send_goal(move_preaction_goal)
        self.right_move_rest_client.wait_for_result()
        move_preaction_result = self.right_move_rest_client.get_result()
        self.left_move_rest_client.send_goal(move_preaction_goal)
        self.left_move_rest_client.wait_for_result()
        move_preaction_result = self.left_move_rest_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Executing actions ...")
        
        for action in action_list:
            source = action["source_object_position"]
            destination = action["target_object_position"]
            prompt = action["label"]
            prompt = String()
            prompt.data = action["label"]
            rospy.loginfo("Sending pick and place goal")
            pick_place_goal = PickPlaceGoal()
            pick_place_goal.source = source
            pick_place_goal.destination = destination
            pick_place_goal.prompt = prompt
            rospy.loginfo(f"Calling pick place with prompt : %s" %pick_place_goal.prompt)
            self.left_pick_place_client.send_goal(pick_place_goal)
            self.left_pick_place_client.wait_for_result()
            pick_place_result = self.left_pick_place_client.get_result()
            print("Pick and place result : ", pick_place_result.result)
            rospy.sleep(1)

        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.left_move_rest_client.send_goal(move_preaction_goal)
        self.left_move_rest_client.wait_for_result()
        move_preaction_result = self.left_move_rest_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Done")

if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    prompt = input("Enter the prompt : ")
    # prompt = "pick the green rectangle using the left arm"

    set_io_client = rospy.ServiceProxy("/left/ur_hardware_interface/set_io", SetIO)
    rospy.sleep(0.05)
    time = rospy.Time.now()
        
    # move to the preaction position
    move_preaction_goal = MovePreactionActionGoal()
    input("Enter to right move to rest")
    central_client.right_move_rest_client.send_goal(move_preaction_goal)
    central_client.right_move_rest_client.wait_for_result()
    move_preaction_result = central_client.right_move_rest_client.get_result()
    input("Enter to left move to look")
    central_client.left_move_look_client.send_goal(move_preaction_goal)
    central_client.left_move_look_client.wait_for_result()
    move_preaction_result = central_client.left_move_look_client.get_result()
    rospy.sleep(0.2)
    # input("Enter to call perception")
    rospy.loginfo("Calling the perception now")
    response = central_client.get_object_locations()
    rospy.loginfo(f"Perception finished in time : {rospy.Time.now() - time}")
    time1 = rospy.Time.now()
    set_io_client(1, 12, 0)
    
    # printing the object id and corresponding classes
    for object_thing in response.result.object_position:
        print("Object ID and class : ", object_thing.id, " ", object_thing.Class)
        print("Object bounding poses : ", object_thing.x_min_y_min, object_thing.x_max_y_max)
    if len(response.result.object_position) == 0:
        rospy.loginfo("No objects detected")
        sys.exit()
    
    input("Enter both move to rest simultaneously")
    central_client.left_move_rest_client.send_goal(move_preaction_goal)
    central_client.left_move_rest_client.wait_for_result()
    central_client.right_move_rest_client.send_goal(move_preaction_goal)
    central_client.right_move_rest_client.wait_for_result()
    
    # save response.result.object_position.image
    annotated_image = cv_bridge.CvBridge().imgmsg_to_cv2(response.result.image, desired_encoding="bgr8")
    cv2.imwrite("/home/barracuda/catkin_ws/src/create_2025_demo/central_scripts/scripts/object_image.png", annotated_image)
    print("Objects detected in time : ", rospy.Time.to_sec(rospy.Time.now()-time))
    time2 = rospy.Time.now()
    plan_actions = central_client.llm(prompt,response.result.object_position,annotated_image)
    object_list_clean = plan_actions["clean"]


    # pick the red ball using the pick client
    for obj in response.result.object_position:
        if obj.Class == "red _ ball":
            print("red ball detected")
            pick_goal = PoseGoal()
            pick_goal.pose = obj.pose
            pick_goal.pose.pose.orientation = RIGHT_ORIENTATION_POSE.pose.orientation
            pick_goal.prompt.data = obj.Class
            print("Pose : ",obj.pose, " class : ",obj.Class)
            input("Pick red ball using right")
            central_client.right_pick_client.send_goal(pick_goal)
            central_client.right_pick_client.wait_for_result()
            pick_result = central_client.right_pick_client.get_result()
            rospy.loginfo(f"Pick result: {pick_result.result}")

    input("Swipe start")
    # for all int in clean_list, clean using the swipe server
    for obj_id in object_list_clean:
        for obj in response.result.object_position:
            if obj.id == obj_id:
                swipe_goal = SwipeGoal()
                swipe_goal.stain_pose = obj.pose
                swipe_goal.stain_pose.pose.orientation = RIGHT_ORIENTATION_POSE.pose.orientation
                swipe_goal.x_min_y_min = obj.x_min_y_min
                swipe_goal.x_max_y_max = obj.x_max_y_max
                central_client.right_swipe_client.send_goal(swipe_goal)
                central_client.right_swipe_client.wait_for_result()
                swipe_result = central_client.right_swipe_client.get_result()
                rospy.loginfo(f"Swipe result: {swipe_result.result}")

    input("Place red ball back to start")
    for obj in response.result.object_position:
        if obj.Class == "red _ ball":
            place_goal = PoseGoal()
            place_goal.pose = obj.pose
            place_goal.pose.pose.orientation = RIGHT_ORIENTATION_POSE.pose.orientation
            central_client.right_place_client.send_goal(place_goal)
            central_client.right_place_client.wait_for_result()
            place_result = central_client.right_place_client.get_result()
            rospy.loginfo(f"Pick result: {pick_result.result}")
    rospy.loginfo(f"Total execution time is {rospy.Time.now()-time}")

    # Move the right arm back to resting position
    rospy.loginfo("Moving right arm back to resting position")
    central_client.right_move_rest_client.send_goal(move_preaction_goal)
    central_client.right_move_rest_client.wait_for_result()
    move_preaction_result = central_client.right_move_rest_client.get_result()
    rospy.loginfo(f"Move preaction result: {move_preaction_result.result}")

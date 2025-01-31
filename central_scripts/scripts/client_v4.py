import rospy
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
from PIL import Image
import io
import base64
import json
import requests
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from create_2025_mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from create_2025_mp_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib
from openai import OpenAI
import numpy as np
from ur_msgs.srv import SetIO

#### Define drope pose #########
DROP_POSE = PoseStamped()
DROP_POSE.pose.position.x= 0.1538011876457178
DROP_POSE.pose.position.y= 0.5658817694268432
DROP_POSE.pose.position.z= 0.15632227392144177
DROP_POSE.pose.orientation.x= -0.7084016817823435
DROP_POSE.pose.orientation.y= 0.7057186070566935
DROP_POSE.pose.orientation.z= 0.007889737191896513
DROP_POSE.pose.orientation.w= 0.008127542614487311
#################################

#### Define pick place orientation #######
ORIENTATION_POSE = PoseStamped()
ORIENTATION_POSE.pose.orientation.x= -0.7084016817823435
ORIENTATION_POSE.pose.orientation.y= 0.7057186070566935
ORIENTATION_POSE.pose.orientation.z= 0.007889737191896513
ORIENTATION_POSE.pose.orientation.w= 0.008127542614487311


#### World Z for different objects 
# blue_circle_dry_z = 0.15199
# green_rectangle_dry_z = 0.1431
# red_triangle_dry_z = 0.1471
# blue_circle_water_z = 0.15199
# green_rectangle_water_z = 0.1431
# red_triangle_water_z = 0.1471
blue_circle_dry_z = 0.1
green_rectangle_dry_z = 0.1
red_triangle_dry_z = 0.1
blue_circle_water_z = 0.1
green_rectangle_water_z = 0.1
red_triangle_water_z = 0.1
##################################

class CentralClient:
    def __init__(self) -> None:
        self.get_object_locations_service = rospy.ServiceProxy(
            "get_object_locations",
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
        rospy.sleep(0.1)
        self.right_pick_place_client.wait_for_server()
        self.right_move_preaction_client.wait_for_server()
        self.left_pick_place_client.wait_for_server()
        self.left_move_preaction_client.wait_for_server()
        self.right_move_look_client.wait_for_server()
        self.right_move_rest_client.wait_for_server()
        self.left_move_look_client.wait_for_server()
        self.left_move_rest_client.wait_for_server()
        rospy.loginfo("All servers are connected")

    def get_object_locations(self):
        try:
            response = self.get_object_locations_service()
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
        preamble = "You are a robot controller, you need to write a sequence of actions. In the image, there are different geometric shapes. You can only execute two types of actions: \"pick_using_left_arm\", \"pick_using_right_arm\", chose the appropriate action for the object depending on the prompt. The output needs to be in the following formats : {\"pick_using_left_arm\":[<object_id1>,<object_id2>, ...],\"pick_using_right_arm\":[<object_id3>, <object_id4>, ... ]}, this output means that the objects_id 1,2,3,4 .... need to be picked up, object id 1,2 .... need to be picked up using left arm and object id 3, 4 .... need to be picked up using right arm. Make sure the output format is adhered, do not include any more description of the reasoning. Refer the image to see which objects are where"
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
        for object in pick_list["pick_using_left_arm"]:
            object = int(object)
        for object in pick_list["pick_using_right_arm"]:
            object = int(object)

        rospy.loginfo(f"The llm returned with object list : {pick_list}")

        return pick_list
        

    # execute all the actions in the action list right one by one here.
    def execute_actions_right(self, action_list):
        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.right_move_look_client.send_goal(move_preaction_goal)
        self.right_move_look_client.wait_for_result()
        move_preaction_result = self.right_move_look_client.get_result()
        self.left_move_rest_client.send_goal(move_preaction_goal)
        self.left_move_rest_client.wait_for_result()
        move_preaction_result = self.left_move_rest_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Executing actions ...")
        
        for action in action_list:
            source = action["source_object_position"]
            destination = action["target_object_position"]
            rospy.loginfo("Sending pick and place goal")
            pick_place_goal = PickPlaceGoal()
            pick_place_goal.source = source
            pick_place_goal.destination = destination
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
            rospy.loginfo("Sending pick and place goal")
            pick_place_goal = PickPlaceGoal()
            pick_place_goal.source = source
            pick_place_goal.destination = destination
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
    set_io_client = rospy.ServiceProxy("/left/ur_hardware_interface/set_io", SetIO)
    set_io_client(1, 13, 1)
    rospy.sleep(0.5)
    time = rospy.Time.now()

    # move to the preaction position
    move_preaction_goal = MovePreactionActionGoal()
    central_client.left_move_rest_client.send_goal(move_preaction_goal)
    central_client.left_move_rest_client.wait_for_result()
    move_preaction_result = central_client.left_move_rest_client.get_result()
    central_client.right_move_look_client.send_goal(move_preaction_goal)
    central_client.right_move_look_client.wait_for_result()
    move_preaction_result = central_client.right_move_look_client.get_result()

    rospy.sleep(0.2)

    rospy.loginfo("Calling the perception now")
    response = central_client.get_object_locations()
    rospy.loginfo(f"Perception finished in time : {rospy.Time.now() - time}")

    time1 = rospy.Time.now()
    set_io_client(1, 12, 0)
    # printing the object id and corresponding classes
    for object_thing in response.result.object_position:
        print("Object ID and class : ", object_thing.id, " ", object_thing.Class)

    # save response.result.object_position.image
    annotated_image = cv_bridge.CvBridge().imgmsg_to_cv2(response.result.image, desired_encoding="bgr8")
    cv2.imwrite("/home/barracuda/catkin_ws/src/create_2025_demo/central_scripts/scripts/object_image.png", annotated_image)
    print("Objects detected in time : ", rospy.Time.to_sec(rospy.Time.now()-time))


    time2 = rospy.Time.now()
    plan_actions = central_client.llm(prompt,response.result.object_position,annotated_image)
    object_list_left = plan_actions["pick_using_left_arm"]
    object_list_right = plan_actions["pick_using_right_arm"]
    
    # create the action list
    action_list_left = []
    action_list_right = []

    # action list 3D
    for object_id in object_list_left:
        source_object_id = object_id
        source_object_position = response.result.object_position[object_id].pose
        if response.result.object_position[object_id].Class == "green _ rectangle":
            source_object_position.pose.position.z = green_rectangle_dry_z
            source_object_position.pose.position.x += 0
            source_object_position.pose.position.y -= 0
        if response.result.object_position[object_id].Class == "red _ triangle":
            source_object_position.pose.position.z = red_triangle_dry_z
            source_object_position.pose.position.x += 0
            source_object_position.pose.position.y -= 0
        if response.result.object_position[object_id].Class == "blue _ circle":
            source_object_position.pose.position.z = blue_circle_dry_z
            source_object_position.pose.position.x += 0
            source_object_position.pose.position.y -= 0
        source_object_position.pose.orientation = ORIENTATION_POSE.pose.orientation
        destination_object_position = DROP_POSE
        action_parsed = {
            "source_object_position": source_object_position,
            "target_object_position": destination_object_position
        }
        action_list_left.append(action_parsed)
    
    # action list in water
    for object_id in object_list_right:
        source_object_id = object_id
        source_object_position = response.result.object_position[object_id].pose
        if response.result.object_position[object_id].Class == "green _ rectangle":
            source_object_position.pose.position.z = green_rectangle_water_z
            source_object_position.pose.position.x += 0
            source_object_position.pose.position.y -= 0
        if response.result.object_position[object_id].Class == "red _ triangle":
            source_object_position.pose.position.z = red_triangle_water_z
            source_object_position.pose.position.x += 0
            source_object_position.pose.position.y -= 0
        if response.result.object_position[object_id].Class == "blue _ circle":
            source_object_position.pose.position.z = blue_circle_water_z
            source_object_position.pose.position.x += 0
            source_object_position.pose.position.y -= 0
        destination_object_position = DROP_POSE
        action_parsed = {
            "source_object_position": source_object_position,
            "target_object_position": destination_object_position
        }
        action_list_right.append(action_parsed)

    # print(action_list_left)
    # print(action_list_right)

    input("Press Enter to continue ...")
    central_client.execute_actions_right(action_list_right)
    central_client.execute_actions_left(action_list_left)
    rospy.loginfo(f"Total execution time is {rospy.Time.now()-time}")

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

#### Define drope pose #########
DROP_POSE = PoseStamped()
DROP_POSE.pose.position.x= 0.2886250627392312
DROP_POSE.pose.position.y= 0.15318942809629893
DROP_POSE.pose.position.z= 0.30891036081855166
DROP_POSE.pose.orientation.x = -0.9625925612708138
DROP_POSE.pose.orientation.y = -0.26923720474112145
DROP_POSE.pose.orientation.z = -0.005001943816401621
DROP_POSE.pose.orientation.w = 0.03003113596485159
#################################

#### World Z for different objects 
blue_circle_z = 0.15199
green_rectangle_z = 0.1431
red_triangle_z = 0.1471
##################################

class CentralClient:
    def __init__(self) -> None:
        self.get_object_locations_service = rospy.ServiceProxy(
            "get_object_locations",
            GetObjectLocations
        )
        self.right_pick_place_client = actionlib.SimpleActionClient("right_pick_place", PickPlaceAction)
        self.right_move_preaction_client = actionlib.SimpleActionClient("right_move_preaction", MovePreactionAction)
        self.left_pick_place_client = actionlib.SimpleActionClient("left_pick_place", PickPlaceAction)
        self.left_move_preaction_client = actionlib.SimpleActionClient("left_move_preaction", MovePreactionAction)
        self.handover_client = actionlib.SimpleActionClient("pick_place_handover", PickPlaceAction)
        self.right_pick_place_client.wait_for_server()
        self.right_move_preaction_client.wait_for_server()
        self.left_pick_place_client.wait_for_server()
        self.left_move_preaction_client.wait_for_server()
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
        preamble = "You are a robot controller, you can choose the objects you can manipulate you can only do one action \"pick\", you need to output in the following way, \"[<object_id_1>,<object_id_2> ...]\", for picking up <object_id_1> and <object_id_2>. Make sure the output format is adhered, do not include any more description of the reasoning. If the robot needs to take any action based on the prompt, use the pick action. Refer the image and use it for spatial reasoning if necessary"
        client = OpenAI()

        completion = client.chat.completions.create(
            model="gpt-4o-mini", 
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
        rospy.loginfo(f"The output of llm : {completion.choices[0].message.content}")

        # Return the generated response
        pick_list = json.loads(completion.choices[0].message.content)
        for object in pick_list:
            object = int(object)


        rospy.loginfo(f"The llm returned with object list : {pick_list}, {type(pick_list[0])}")

        return pick_list
        


    # execute all the actions in the action list one by one here.
    def execute_actions(self, action_list):
        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.right_move_preaction_client.send_goal(move_preaction_goal)
        self.right_move_preaction_client.wait_for_result()
        move_preaction_result = self.right_move_preaction_client.get_result()
        self.left_move_preaction_client.send_goal(move_preaction_goal)
        self.left_move_preaction_client.wait_for_result()
        move_preaction_result = self.left_move_preaction_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Executing actions ...")
        
        for action in action_list:
            source = action["source_object_position"]
            destination = action["target_object_position"]
            rospy.loginfo("Sending pick and place goal")
            pick_place_goal = PickPlaceGoal()
            pick_place_goal.source = source
            pick_place_goal.destination = destination
            self.handover_client.send_goal(pick_place_goal)
            self.handover_client.wait_for_result()
            pick_place_result = self.handover_client.get_result()
            print("Pick and place result : ", pick_place_result.result)
            rospy.sleep(1)

        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.right_move_preaction_client.send_goal(move_preaction_goal)
        self.right_move_preaction_client.wait_for_result()
        move_preaction_result = self.right_move_preaction_client.get_result()
        self.left_move_preaction_client.send_goal(move_preaction_goal)
        self.left_move_preaction_client.wait_for_result()
        move_preaction_result = self.left_move_preaction_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)

        print("Done")

if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    # prompt = input("Enter the prompt : ")
    
    time = rospy.Time.now()
    # move to the preaction position
    move_preaction_goal = MovePreactionActionGoal()
    central_client.right_move_preaction_client.send_goal(move_preaction_goal)
    central_client.right_move_preaction_client.wait_for_result()
    move_preaction_result = central_client.right_move_preaction_client.get_result()
    central_client.left_move_preaction_client.send_goal(move_preaction_goal)
    central_client.left_move_preaction_client.wait_for_result()
    move_preaction_result = central_client.left_move_preaction_client.get_result()

    rospy.loginfo("Calling the perception now")
    response = central_client.get_object_locations()
    rospy.loginfo(f"Perception finished in time : {rospy.Time.now() - time}")

    time1 = rospy.Time.now()
    # printing the object id and corresponding classes
    for object_thing in response.result.object_position:
        print("Object ID : ", object_thing.id)
        print("Object Class : ", object_thing.Class)
        print("Object_pose : ",object_thing.pose)

    # save response.result.object_position.image
    annotated_image = cv_bridge.CvBridge().imgmsg_to_cv2(response.result.image, desired_encoding="bgr8")
    cv2.imwrite("/home/barracuda/catkin_ws/src/create_2025_demo/central_scripts/scripts/object_image.png", annotated_image)
    print("Objects detected in time : ", rospy.Time.to_sec(rospy.Time.now()-time))


    time2 = rospy.Time.now()
    # object_list = central_client.llm(prompt,response.result.object_position,annotated_image)
    
    object_list = []
    while True : 
        id = int(input("enter the id, 5 to cancel : "))
        if id == 5:
            break
        object_list.append(id)

    # create the action list
    action_list = []

    for object_id in object_list:
        source_object_id = object_id
        source_object_position = response.result.object_position[object_id].pose
        if response.result.object_position[object_id].Class == "green _ rectangle":
            source_object_position.pose.position.z = green_rectangle_z - 0.004
            source_object_position.pose.position.x += 0.03
            source_object_position.pose.position.y -= 0.04
        if response.result.object_position[object_id].Class == "red _ triangle":
            source_object_position.pose.position.z = red_triangle_z - 0.007
            source_object_position.pose.position.x += 0.03
            source_object_position.pose.position.y -= 0.04
        if response.result.object_position[object_id].Class == "blue _ circle":
            source_object_position.pose.position.z = blue_circle_z - 0.0075
            source_object_position.pose.position.x += 0.03
            source_object_position.pose.position.y -= 0.04
        destination_object_position = DROP_POSE
        action_parsed = {
            "source_object_position": source_object_position,
            "target_object_position": destination_object_position
        }
        action_list.append(action_parsed)

    input("Press Enter to continue ...")
    central_client.execute_actions(action_list)
    rospy.loginfo(f"Total execution time is {rospy.Time.now()-time}")

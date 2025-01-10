import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from typing import List, Tuple
import cv_bridge, cv2
import image_geometry
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from groundingdino.util.inference import load_model, load_image, predict
from open_set_object_detection_msgs.msg import ObjectPosition, ObjectPositions
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import numpy as np
import supervision as sv
import torch
from PIL import Image as PILImage
from torchvision.ops import box_convert
import tf2_ros, tf2_geometry_msgs
import bisect
import os
####### model parameters threshold ########
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
TEXT_PROMPT = "detect .green_rectangle.red_triangle.blue_circle. in the white space, there are only 4 objects"
####################################### 


class Deprojection:

    def __init__(self) -> None:
        
        self.depth_image = None # contains the depth images
        self.camera_info = None # contains the camera info
        self.color_image = None # contains the color image
        self.camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()
        self.latest_result = None
        self.recursion = 0
        self.source_image_path = "assets/generated_image.jpeg"
        self.model = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")
        rospy.loginfo("Loaded the Grounding Dino model")

        # Subscribers
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
        
        # Service
        server = rospy.Service("get_object_locations", GetObjectLocations, self.get_object_locations)
        
        # create a publisher to publish the stream
        self.stream_pub = rospy.Publisher("/orange_position", PoseStamped, queue_size=10)

        # create a timer to publish stream
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_stream)
        pass

    # create a desctructor
    def __del__(self):
        
        del self.model

        pass

    def annotate(self, image_source: np.ndarray, boxes: torch.Tensor, logits: torch.Tensor, phrases: List[str]) -> np.ndarray:
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()
        detections = sv.Detections(xyxy=xyxy)

        # Add IDs and adjust labels
        labels = [
            f"{i}: {phrase} {logit:.2f}"
            for i, (phrase, logit) in enumerate(zip(phrases, logits))
        ]

        # Annotators with larger font size
        bbox_annotator = sv.BoxAnnotator(color_lookup=sv.ColorLookup.INDEX)
        label_annotator = sv.LabelAnnotator(
            color_lookup=sv.ColorLookup.INDEX, 
            # text_properties={"font_scale": 1.5}  # Adjust font scale here
        )

        # Annotate the image
        annotated_frame = cv2.cvtColor(image_source, cv2.COLOR_RGB2BGR)
        annotated_frame = bbox_annotator.annotate(scene=annotated_frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections, labels=labels)
        return annotated_frame

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

    def publish_stream(self, event):
        if self.latest_result is None:
            return
        position = PoseStamped()
        position.header.frame_id = "world"
        position.header.stamp = rospy.Time.now()
        position.pose = self.latest_result.object_position[0].pose.pose
        self.stream_pub.publish(position)
        pass


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
        
        # return pose

    # this is the server function
    def get_object_locations(self,request):
        
        # save the current image as a source image
        # change the image from bgr to rgb
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.source_image_path, self.color_image)

        # execute Grounding Dino here to get the bounding boxes based on the prompt
        image_path = self.source_image_path
        text_prompt = TEXT_PROMPT
        box_threshold = BOX_THRESHOLD
        text_threshold = TEXT_THRESHOLD
        image_source, image = load_image(image_path)

        boxes, logits, phrases = predict(
            model=self.model,
            image=image,
            caption=text_prompt,
            box_threshold=box_threshold,
            text_threshold=text_threshold
        )

        annotated_frame = self.annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
        cv2.imwrite("inference_images/annotated_image.jpg", annotated_frame)

        result = ObjectPositions()

        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy().astype(int)
        detections = sv.Detections(xyxy=xyxy)

        labels = [
            f"{phrase} {logit:.2f}"
            for phrase, logit
            in zip(phrases, logits)
        ]

        print("########################")
        print("\nBoxes : ")
        print(boxes, type(boxes))
        print("\nBoxes : ")
        print(xyxy, type(xyxy))
        print("\nLabels : ")
        print(labels, type(labels))
        print("\nLabels : ")
        print(phrases, type(phrases))
        print("\n")
        print("########################")


        # PREPARE THE RESULT OBJECT HERE
        for i in range(len(phrases)):
            object_position = ObjectPosition()
            object_position.id = i
            object_position.Class = phrases[i]
            x = int((xyxy[i][0] + xyxy[i][2]) /2)
            y = int((xyxy[i][1] + xyxy[i][3]) /2)
            pose = self.get_3d_position(x, y)
            if pose is None:
                return
            object_position.pose = pose
            object_position.x_min = xyxy[i][0]
            object_position.y_min = xyxy[i][1]
            object_position.x_max = xyxy[i][2]
            object_position.y_max = xyxy[i][3]
            result.object_position.append(object_position)

        result.image = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.latest_result = result

        return GetObjectLocationsResponse(result)
    
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


if __name__ == "__main__":
    rospy.init_node("deprojection_node")
    rospy.sleep(0.5)
    deproject = Deprojection()
    rospy.sleep(0.5)
    rospy.loginfo("Deproject server ready...")
    rospy.spin()
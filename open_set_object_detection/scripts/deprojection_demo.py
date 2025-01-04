import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
import cv_bridge, cv2
import image_geometry
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from groundingdino.util.inference import load_model, load_image, predict, annotate
from open_set_object_detection_msgs.msg import ObjectPosition, ObjectPositions
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import numpy as np
import supervision as sv
import torch
from PIL import Image as PILImage
from torchvision.ops import box_convert
import bisect

####### model parameters threshold ########
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
TEXT_PROMPT = "face"
######################################

class Deprojection:

    def __init__(self) -> None:
        
        self.depth_image = None # contains the depth images
        self.camera_info = None # contains the camera info
        self.color_image = None # contains the color image
        self.camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()
        self.latest_result = None
        self.source_image_path = "assets/generated_image.jpeg"
        self.model = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")
        rospy.loginfo("Loaded the Grounding Dino model")

        # Subscribers
        self.depth_image_sub = rospy.Subscriber(
                                                "/depth_to_rgb/image", 
                                                Image, 
                                                self.depth_image_callback
                                                )
        self.camera_info_sub = rospy.Subscriber(
                                                "/rgb/camera_info", 
                                                CameraInfo, 
                                                self.camera_info_callback
                                                )
        self.color_image_sub = rospy.Subscriber(
                                                "/rgb/image_raw",
                                                Image,
                                                self.color_image_callback
                                                )
        
        # Service
        server = rospy.Service("get_object_locations", GetObjectLocations, self.get_object_locations)
        
        # # create a publisher to publish the stream
        # self.stream_pub = rospy.Publisher("/orange_position", PointStamped, queue_size=10)

        # # create a timer to publish stream
        # self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_stream)
        pass

    # create a desctructor
    def __del__(self):
        
        del self.model

        pass


    # def publish_stream(self, event):
    #     if self.bounding_boxes is None:
    #         return
    #     position = PointStamped()
    #     for box in self.bounding_boxes:
    #         id_ = box.id
    #         class_ = box.Class
    #         x = int((box.xmin + box.xmax) /2)
    #         y = int((box.ymin + box.ymax) /2)
    #         position = self.get_3d_position(x, y)
    #         if position is None:
    #             return
    #     self.stream_pub.publish(position)
    #     print("Published")


    def get_3d_position(self, x, y):
        if self.depth_image is None and self.camera_info is None:
            return  # Wait until depth image is received
        depth = (self.depth_image[y, x])/1000  # Convert to meters
        if np.isnan(depth) or depth == 0:
            rospy.logwarn("Invalid depth at pixel ({}, {})".format(x, y))
            return
        # Project the 2D pixel to 3D point in the camera frame
        point_3d = self.camera_model.projectPixelTo3dRay((x, y))
        point_3d = np.array(point_3d) * depth  # Scale the ray by the depth
        pose = PoseStamped()
        pose.header.frame_id = self.camera_model.tf_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point_3d[0]
        pose.pose.position.y = point_3d[1]
        pose.pose.position.z = point_3d[2]
        # set pose orientation quaternion here

        return pose
        

    # this is the server function
    def get_object_locations(self,request):
        
        # save the current image as a source image
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

        annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
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
        print("\n")

        print("########################")
        print("\nBoxes : ")
        print(xyxy, type(xyxy))
        print("\n")

        print("########################")
        print("\nLabels : ")
        print(labels, type(labels))
        print("\n")

        print("########################")
        print("\nLabels : ")
        print(phrases, type(phrases))
        print("\n")


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
    deproject = Deprojection()
    rospy.sleep(0.5)
    rospy.loginfo("Deproject server ready...")
    rospy.spin()
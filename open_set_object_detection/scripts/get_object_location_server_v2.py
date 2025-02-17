import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from typing import List
import cv_bridge, cv2
import image_geometry
import numpy as np
from groundingdino.util.inference import load_model, load_image, predict
from open_set_object_detection_msgs.msg import ObjectPosition, ObjectPositions
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import supervision as sv
import torch
from torchvision.ops import box_convert
import tf2_ros, tf2_geometry_msgs

# Model parameters
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
TEXT_PROMPT = "blue_circle.red_triangle.green_square"

# Camera identifiers
LEFT_CAMERA = "rs_415_left"
RIGHT_CAMERA = "rs_415_right"

class Deprojection:
    def __init__(self) -> None:
        # Initialize camera-related variables
        self.left_depth_image = None
        self.left_camera_info = None
        self.left_color_image = None
        self.right_depth_image = None
        self.right_camera_info = None
        self.right_color_image = None
        self.left_bbox = [0, 0, 0, 0]  # Default values for left bounding box
        self.right_bbox = [0, 0, 0, 0]  # Default values for right bounding box
        self.left_camera_model = image_geometry.PinholeCameraModel()
        self.right_camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()
        self.latest_result = None
        self.recursion = 0
        self.source_image_path = "assets/generated_image.jpeg"
        # Load the Grounding Dino model
        self.model = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")
        rospy.loginfo("Loaded the Grounding Dino model")

        # Camera topics
        left_camera_color_topic = f"/left/{LEFT_CAMERA}/color/image_raw"
        left_camera_info_topic = f"/left/{LEFT_CAMERA}/aligned_depth_to_color/camera_info"
        left_camera_depth_topic = f"/left/{LEFT_CAMERA}/aligned_depth_to_color/image_raw"
        right_camera_color_topic = f"/right/{RIGHT_CAMERA}/color/image_raw"
        right_camera_info_topic = f"/right/{RIGHT_CAMERA}/aligned_depth_to_color/camera_info"
        right_camera_depth_topic = f"/right/{RIGHT_CAMERA}/aligned_depth_to_color/image_raw"
        
        # Workspace bounding box topics
        left_bbox_topic = f"/left/{LEFT_CAMERA}/workspace"
        right_bbox_topic = f"/right/{RIGHT_CAMERA}/workspace"

        # Fetch depth thresholds
        self.left_depth_threshold = rospy.get_param(f"/left/{LEFT_CAMERA}/depth_ws_threshold", 1000)
        self.right_depth_threshold = rospy.get_param(f"/right/{RIGHT_CAMERA}/depth_ws_threshold", 1000)

        # Subscribers
        self.left_depth_image_sub = rospy.Subscriber(left_camera_depth_topic, Image, self.left_depth_image_callback)
        self.left_camera_info_sub = rospy.Subscriber(left_camera_info_topic, CameraInfo, self.left_camera_info_callback)
        self.left_color_image_sub = rospy.Subscriber(left_camera_color_topic, Image, self.left_color_image_callback)
        self.right_depth_image_sub = rospy.Subscriber(right_camera_depth_topic, Image, self.right_depth_image_callback)
        self.right_camera_info_sub = rospy.Subscriber(right_camera_info_topic, CameraInfo, self.right_camera_info_callback)
        self.right_color_image_sub = rospy.Subscriber(right_camera_color_topic, Image, self.right_color_image_callback)

        # Subscribers for workspace bounding boxes
        self.left_bbox_sub = rospy.Subscriber(left_bbox_topic, Float32MultiArray, self.left_workspace_callback)
        self.right_bbox_sub = rospy.Subscriber(right_bbox_topic, Float32MultiArray, self.right_workspace_callback)

        # Services
        rospy.Service("left_get_object_locations", GetObjectLocations, self.left_get_object_locations)
        rospy.Service("right_get_object_locations", GetObjectLocations, self.right_get_object_locations)

        # Publisher for streamed position
        self.stream_pub = rospy.Publisher("/orange_position", PoseStamped, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_stream)

    def __del__(self):
        del self.model

    def left_workspace_callback(self, msg: Float32MultiArray):
        self.left_bbox = msg.data

    def right_workspace_callback(self, msg: Float32MultiArray):
        self.right_bbox = msg.data

    def annotate(self, image_source: np.ndarray, boxes: torch.Tensor, logits: torch.Tensor, phrases: List[str]) -> np.ndarray:
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()
        detections = sv.Detections(xyxy=xyxy)
        labels = [f"{i}: {phrase} {logit:.2f}" for i, (phrase, logit) in enumerate(zip(phrases, logits))]
        bbox_annotator = sv.BoxAnnotator(color_lookup=sv.ColorLookup.INDEX)
        label_annotator = sv.LabelAnnotator(color_lookup=sv.ColorLookup.INDEX)
        annotated_frame = cv2.cvtColor(image_source, cv2.COLOR_RGB2BGR)
        annotated_frame = bbox_annotator.annotate(scene=annotated_frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections, labels=labels)
        return annotated_frame

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        try:
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            tf_buffer.can_transform(target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                pose,
                tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rospy.Time(0))
            )
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")
            return None

    def publish_stream(self, event):
        if self.latest_result is None:
            return
        position = PoseStamped()
        position.header.frame_id = "world"
        position.header.stamp = rospy.Time.now()
        position.pose = self.latest_result.object_position[0].pose.pose
        self.stream_pub.publish(position)

    def get_3d_position(self, x, y, depth_image, camera_info, camera_model, depth_threshold):
        if depth_image is None or camera_info is None:
            return None
        
        depth = depth_image[y, x] / 1000  # Convert to meters
        if np.isnan(depth) or depth == 0 or depth > depth_threshold / 1000.0:  # Validate using threshold
            rospy.logwarn(f"Invalid depth at pixel ({x}, {y}) or above threshold.")
            self.recursion += 1
            if self.recursion <= 10:
                return self.get_3d_position(x, y, depth_image=depth_image, camera_info=camera_info, camera_model=camera_model, depth_threshold=depth_threshold)
            else:
                self.recursion = 0
                return None

        point_3d = np.array(camera_model.projectPixelTo3dRay((x, y))) * depth
        pose = PoseStamped()
        pose.header.frame_id = camera_model.tf_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point_3d
        pose.pose.orientation.w = 1.0
        return self.transform_pose(pose, "world")

    def left_get_object_locations(self, request):
        if self.left_color_image is None or not any(self.left_bbox):
            return GetObjectLocationsResponse()
        
        x_min, x_max, y_min, y_max = map(int, self.left_bbox)
        cropped_color_image = self.left_color_image[y_min:y_max, x_min:x_max]

        color_image = cv2.cvtColor(cropped_color_image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.source_image_path, color_image)

        image_source, image = load_image(self.source_image_path)
        boxes, logits, phrases = predict(
            model=self.model,
            image=image,
            caption=request.prompt.data,
            box_threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD
        )
        annotated_frame = self.annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
        cv2.imwrite("inference_images/annotated_image_left.jpg", annotated_frame)

        result = ObjectPositions()
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy().astype(int)

        for i in range(len(phrases)):
            object_position = ObjectPosition()
            object_position.id = i
            object_position.Class = phrases[i]
            x_center = int((xyxy[i][0] + xyxy[i][2]) / 2) + x_min
            y_center = int((xyxy[i][1] + xyxy[i][3]) / 2) + y_min
            pose = self.get_3d_position(x_center, y_center, depth_image=self.left_depth_image, camera_info=self.left_camera_info, camera_model=self.left_camera_model, depth_threshold=self.left_depth_threshold)
            if pose is None:
                continue
            object_position.pose = pose
            object_position.x_min = xyxy[i][0] + x_min
            object_position.y_min = xyxy[i][1] + y_min
            object_position.x_max = xyxy[i][2] + x_min
            object_position.y_max = xyxy[i][3] + y_min
            result.object_position.append(object_position)

        result.image = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.latest_result = result
        return GetObjectLocationsResponse(result)

    def right_get_object_locations(self, request):
        if self.right_color_image is None or not any(self.right_bbox):
            return GetObjectLocationsResponse()
        
        x_min, x_max, y_min, y_max = map(int, self.right_bbox)
        cropped_color_image = self.right_color_image[y_min:y_max, x_min:x_max]

        color_image = cv2.cvtColor(cropped_color_image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(self.source_image_path, color_image)

        image_source, image = load_image(self.source_image_path)
        boxes, logits, phrases = predict(
            model=self.model,
            image=image,
            caption=request.prompt.data,
            box_threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD
        )
        annotated_frame = self.annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
        cv2.imwrite("inference_images/annotated_image_right.jpg", annotated_frame)

        result = ObjectPositions()
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy().astype(int)

        for i in range(len(phrases)):
            object_position = ObjectPosition()
            object_position.id = i
            object_position.Class = phrases[i]
            x_center = int((xyxy[i][0] + xyxy[i][2]) / 2) + x_min
            y_center = int((xyxy[i][1] + xyxy[i][3]) / 2) + y_min
            pose = self.get_3d_position(x_center, y_center, depth_image=self.right_depth_image, camera_info=self.right_camera_info, camera_model=self.right_camera_model, depth_threshold=self.right_depth_threshold)
            if pose is None:
                continue
            object_position.pose = pose
            object_position.x_min = xyxy[i][0] + x_min
            object_position.y_min = xyxy[i][1] + y_min
            object_position.x_max = xyxy[i][2] + x_min
            object_position.y_max = xyxy[i][3] + y_min
            result.object_position.append(object_position)

        result.image = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.latest_result = result
        return GetObjectLocationsResponse(result)

    def left_color_image_callback(self, msg: Image):
        self.left_color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def left_depth_image_callback(self, msg: Image):
        self.left_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def left_camera_info_callback(self, msg: CameraInfo):
        self.left_camera_info = msg
        self.left_camera_model.fromCameraInfo(msg)

    def right_color_image_callback(self, msg: Image):
        self.right_color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def right_depth_image_callback(self, msg: Image):
        self.right_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def right_camera_info_callback(self, msg: CameraInfo):
        self.right_camera_info = msg
        self.right_camera_model.fromCameraInfo(msg)

if __name__ == "__main__":
    rospy.init_node("deprojection_node")
    rospy.sleep(0.5)
    deproject = Deprojection()
    rospy.sleep(0.5)
    rospy.loginfo("Deproject server ready...")
    rospy.spin()
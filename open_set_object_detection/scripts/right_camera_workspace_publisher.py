#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class DepthOverlayNode:
    def __init__(self, arm_namespace, camera):
        rospy.init_node(f"{arm_namespace}_{camera}_depth_overlay_node", anonymous=True)

        # Topics for depth and color images
        self.depth_topic = f"/{arm_namespace}/{camera}/aligned_depth_to_color/image_raw"
        self.color_topic = f"/{arm_namespace}/{camera}/color/image_raw"
        self.overlay_topic = f"/{arm_namespace}/{camera}/color/workspace_image_raw"
        self.workspace_bbox_topic = f"/{arm_namespace}/{camera}/workspace"

        # ROS param name for depth threshold
        self.depth_threshold_param = f"/{arm_namespace}/{camera}/depth_ws_threshold"

        # Get initial depth threshold (default 1000 mm)
        self.depth_threshold = rospy.get_param(self.depth_threshold_param, 1000)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to depth and color topics
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        self.color_sub = rospy.Subscriber(self.color_topic, Image, self.color_callback)

        # Publisher for overlayed image
        self.overlay_pub = rospy.Publisher(self.overlay_topic, Image, queue_size=1)

        # Publisher for workspace bounding box
        self.bbox_pub = rospy.Publisher(self.workspace_bbox_topic, Float32MultiArray, queue_size=1)

        # Image storage
        self.color_image = None
        self.depth_image = None

        # Bounding box of the non-red workspace
        self.x_min, self.x_max, self.y_min, self.y_max = None, None, None, None

        # Start separate threads for updating threshold and publishing bounding box
        self.threshold_update_thread = threading.Thread(target=self.update_threshold, daemon=True)
        self.threshold_update_thread.start()

        # Timer for publishing the workspace bounding box at 30 Hz
        rospy.Timer(rospy.Duration(1.0/30.0), self.publish_workspace_bbox)

    def update_threshold(self):
        """ Asynchronously updates the depth threshold every 1 second (1 Hz). """
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.depth_threshold = rospy.get_param(self.depth_threshold_param, 1000)
            rate.sleep()

    def depth_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format (16-bit single-channel image)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Convert depth to float and filter out invalid values
            depth_image = depth_image.astype(np.float32)
            depth_image[depth_image == 0] = np.nan  # Set invalid depths to NaN

            self.depth_image = depth_image  # Store depth image

            # Publish overlay image if both images are available
            self.publish_overlay()

        except Exception as e:
            rospy.logerr(f"[{self.depth_topic}] Error processing depth image: {e}")

    def color_callback(self, msg):
        try:
            # Convert ROS color image to OpenCV format (BGR)
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.color_image = color_image

            # Publish overlay image if both images are available
            self.publish_overlay()

        except Exception as e:
            rospy.logerr(f"[{self.color_topic}] Error processing color image: {e}")

    def publish_overlay(self):
        if self.color_image is None or self.depth_image is None:
            return  # Wait until both images are received

        # Create a mask where depth > threshold or is NaN
        mask = (self.depth_image > self.depth_threshold) | np.isnan(self.depth_image)

        # Convert BGR image to RGB
        color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        # Overlay red color on the mask
        color_image[mask] = [255, 0, 0]  # Red for high depth or NaN

        # Compute the bounding box of non-red workspace
        self.compute_workspace_bbox(mask)

        # Draw the bounding box in blue
        if None not in (self.x_min, self.x_max, self.y_min, self.y_max):
            cv2.rectangle(color_image, (self.x_min, self.y_min), (self.x_max, self.y_max), (0, 0, 255), 2)

        # Convert back to ROS Image message
        overlay_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="rgb8")

        # Publish the overlay image
        self.overlay_pub.publish(overlay_msg)

    def compute_workspace_bbox(self, mask):
        """ Computes the bounding box (x_min, x_max, y_min, y_max) of the non-red workspace. """
        non_red_pixels = np.argwhere(~mask)  # Get coordinates of non-red pixels

        if non_red_pixels.size == 0:
            self.x_min, self.x_max, self.y_min, self.y_max = None, None, None, None
            rospy.logwarn(f"[{self.overlay_topic}] No valid workspace detected!")
            return

        self.y_min, self.x_min = np.min(non_red_pixels, axis=0)
        self.y_max, self.x_max = np.max(non_red_pixels, axis=0)

        # rospy.loginfo(f"[{self.overlay_topic}] Workspace Bounding Box: "
        #               f"x_min={self.x_min}, x_max={self.x_max}, "
        #               f"y_min={self.y_min}, y_max={self.y_max}")

    def publish_workspace_bbox(self, event):
        """ Publishes the workspace bounding box coordinates as a Float32MultiArray. """
        if None not in (self.x_min, self.x_max, self.y_min, self.y_max):
            bbox_msg = Float32MultiArray()
            bbox_msg.data = [float(self.x_min), float(self.x_max), float(self.y_min), float(self.y_max)]
            self.bbox_pub.publish(bbox_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    left_node = DepthOverlayNode("right", "rs_415_right")
    rospy.loginfo(f"[{left_node.depth_topic}] Depth Overlay Node is running with threshold: {left_node.depth_threshold} mm")
    rospy.spin()
#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy import stats
from collections import Counter

class DepthStatsNode:
    def __init__(self):
        rospy.init_node("depth_statistics_node", anonymous=True)

        # Topics
        self.depth_topic = "/left/rs_415_left/aligned_depth_to_color/image_raw"
        self.color_topic = "/left/rs_415_left/color/image_raw"

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to topics
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        self.color_sub = rospy.Subscriber(self.color_topic, Image, self.color_callback)

        # Depth statistics
        self.min_depth = None
        self.max_depth = None
        self.mean_depth = None
        self.median_depth = None
        self.std_dev_depth = None
        self.mode_depth = None
        self.valid_depths = None

        # Image storage
        self.color_image = None
        self.depth_image = None
        self.data_received = False

    def depth_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format (16-bit single-channel image)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Convert depth to float and filter out invalid values
            depth_image = depth_image.astype(np.float32)
            depth_image[depth_image == 0] = np.nan  # Set invalid depths to NaN

            self.depth_image = depth_image  # Store depth image

            # Extract valid depth values (0-2000 mm)
            valid_depths = depth_image[(depth_image > 0) & (depth_image <= 2000)].flatten()

            if valid_depths.size == 0:
                rospy.logwarn("No valid depth values found within range 0-2000 mm!")
                return
            
            # Compute statistics
            self.min_depth = np.nanmin(valid_depths)
            self.max_depth = np.nanmax(valid_depths)
            self.mean_depth = np.nanmean(valid_depths)
            self.median_depth = np.nanmedian(valid_depths)
            self.std_dev_depth = np.nanstd(valid_depths)
            self.mode_depth = stats.mode(valid_depths, keepdims=True)[0][0]
            self.valid_depths = valid_depths

            self.data_received = True  # Indicate valid data received

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def color_callback(self, msg):
        try:
            # Convert ROS color image to OpenCV format (BGR)
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.color_image = color_image

        except Exception as e:
            rospy.logerr(f"Error processing color image: {e}")

    def overlay_depth_on_color(self):
        if self.color_image is None or self.depth_image is None:
            rospy.logwarn("Waiting for both color and depth images...")
            return None
        
        # Create a mask where depth > 750 mm or is NaN
        mask = (self.depth_image > 1000) | np.isnan(self.depth_image)

        # Convert BGR image to RGB
        color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        # Overlay red color on the mask
        color_image[mask] = [255, 0, 0]  # Red color for high depth or NaN

        return color_image

    def run(self):
        rospy.loginfo("Waiting for valid depth and color data...")
        while not rospy.is_shutdown() and not self.data_received:
            rospy.sleep(0.5)

        if rospy.is_shutdown():
            return

        rospy.loginfo(f"Depth Stats - Min: {self.min_depth:.2f}, Max: {self.max_depth:.2f}, Mean: {self.mean_depth:.2f}, "
                      f"Mode: {self.mode_depth:.2f}, Median: {self.median_depth:.2f}, Std Dev: {self.std_dev_depth:.2f}")

        # Compute depth distribution (pixel count per depth value)
        depth_counts = Counter(self.valid_depths)
        depths = np.array(sorted(depth_counts.keys()))  # Unique depth values (x-axis)
        pixel_counts = np.array([depth_counts[d] for d in depths])  # Number of pixels (y-axis)

        # Plot depth values as a continuous line plot with x-axis limited to 0-2000 mm
        plt.figure(figsize=(8, 6))
        plt.plot(depths, pixel_counts, color='blue', linewidth=2)
        plt.xlabel("Depth (mm)")
        plt.ylabel("Number of Pixels")
        plt.title("Depth Distribution (0-2000 mm)")
        plt.xlim(0, 2000)  # Limit x-axis range
        plt.grid(True)
        plt.show()

        # Overlay red on color image
        overlayed_image = self.overlay_depth_on_color()
        if overlayed_image is not None:
            plt.figure(figsize=(8, 6))
            plt.imshow(overlayed_image)
            plt.title("Color Image with Depth Overlay (>750 mm in Red)")
            plt.axis("off")
            plt.show()

        rospy.spin()

if __name__ == "__main__":
    node = DepthStatsNode()
    node.run()

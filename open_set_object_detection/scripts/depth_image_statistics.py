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

        # Depth topic
        self.depth_topic = "/left/rs_415_left/aligned_depth_to_color/image_raw"
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to depth topic
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)

        # Depth statistics
        self.min_depth = None
        self.max_depth = None
        self.mean_depth = None
        self.median_depth = None
        self.std_dev_depth = None
        self.mode_depth = None
        self.valid_depths = None

        # Ensure at least one valid frame is received before proceeding
        self.data_received = False

    def depth_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format (16-bit single-channel image)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Remove invalid depths (0 values)
            valid_depths = depth_image[depth_image > 0].flatten()

            if valid_depths.size == 0:
                rospy.logwarn("No valid depth values found!")
                return
            
            # Compute statistics
            self.min_depth = np.min(valid_depths)
            self.max_depth = np.max(valid_depths)
            self.mean_depth = np.mean(valid_depths)
            self.median_depth = np.median(valid_depths)
            self.std_dev_depth = np.std(valid_depths)
            self.mode_depth = stats.mode(valid_depths, keepdims=True)[0][0]
            self.valid_depths = valid_depths

            # Indicate that valid data has been received
            self.data_received = True

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def run(self):
        # Wait until valid depth data is received
        rospy.loginfo("Waiting for valid depth data...")
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

        # Plot depth values as a continuous line plot
        plt.figure(figsize=(8, 6))
        plt.plot(depths, pixel_counts, color='blue', linewidth=2)
        plt.xlabel("Depth (mm)")
        plt.ylabel("Number of Pixels")
        plt.title("Depth Distribution (Continuous)")
        plt.xlim(0,2000)
        plt.grid(True)
        plt.show()

        rospy.spin()

if __name__ == "__main__":
    node = DepthStatsNode()
    node.run()

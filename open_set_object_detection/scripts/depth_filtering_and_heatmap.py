#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

class DepthFillingNode:
    def __init__(self):
        rospy.init_node("depth_filling_node", anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/right/rs_435i/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.heatmap_pub_raw = rospy.Publisher("/right/rs_435i/aligned_depth_to_color/heatmap_raw", Image, queue_size=1)
        self.heatmap_pub_filled = rospy.Publisher("/right/rs_435i/aligned_depth_to_color/heatmap_filled", Image, queue_size=1)
        rospy.loginfo("Depth Filling Node Started")
    
    def depth_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Normalize depth image for visualization
            depth_vis = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Fill missing depth values using Navier-Stokes inpainting
            mask = (depth_image == 0).astype(np.uint8)  # Mask for missing depth values
            filled_depth = cv2.inpaint(depth_vis, mask, inpaintRadius=3, flags=cv2.INPAINT_NS)
            
            # Generate heatmaps
            heatmap_raw = self.generate_heatmap(depth_vis)
            heatmap_filled = self.generate_heatmap(filled_depth)
            
            # Publish heatmaps
            self.heatmap_pub_raw.publish(self.bridge.cv2_to_imgmsg(heatmap_raw, encoding="bgr8"))
            self.heatmap_pub_filled.publish(self.bridge.cv2_to_imgmsg(heatmap_filled, encoding="bgr8"))
            
        except Exception as e:
            rospy.logerr("Error processing depth image: {}".format(e))
    
    def generate_heatmap(self, depth_image):
        # Clip depth values to 4m range and scale for colormap
        depth_clipped = np.clip(depth_image, 0, 4.0)
        depth_scaled = ((depth_clipped / 4.0) * 255).astype(np.uint8)
        colormap = cv2.applyColorMap(depth_scaled, cv2.COLORMAP_JET)
        return colormap

if __name__ == "__main__":
    try:
        DepthFillingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
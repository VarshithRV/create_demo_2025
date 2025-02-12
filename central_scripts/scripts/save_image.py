#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import os, sys


class SaveImage:
    def __init__(self)->None:
        
        # create a path for images in the ../images folder
        self.path = os.path.join(sys.path[0],"../images")
        self.rgb_image = None
        self.depth_image = None
        self.depth_camera_info = None
        self.color_camera_info = None
        self.bridge = cv_bridge.CvBridge()

        self.rgb_image_subscriber = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            callback=self.rgb_image_callback
        )
        self.depth_image_subscriber = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image,
            callback=self.depth_image_callback
        )
        self.depth_camera_info_subscriber = rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            callback=self.depth_camera_info_callback
        )
        self.color_camera_info_subscriber = rospy.Subscriber(
            "/camera/color/camera_info",
            CameraInfo,
            callback=self.color_camera_info_callback
        )


    def rgb_image_callback(self,msg:Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def depth_image_callback(self,msg:Image):
        self.depth_image = (self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough"))

    def color_camera_info_callback(self,msg:Image):
        self.color_camera_info = msg
    
    def depth_camera_info_callback(self,msg:Image):
        self.depth_camera_info = msg


if __name__ == "__main__":
    rospy.init_node("save_image")
    save_img = SaveImage()
    rospy.sleep(0.5)
    # write it in the images folder
    cv2.imwrite("rgb_image.jpg",save_img.rgb_image)

    
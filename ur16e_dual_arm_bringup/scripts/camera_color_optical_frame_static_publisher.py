import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import os
import pickle

class CameraColorOpticalFrameStaticPublisher:
    def __init__(self)->None:
        self.static_broadcaster_camera_color_optical_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.map_base_link_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.camera_color_optical_frame_base_link = TransformStamped()
        self.camera_color_optical_frame_base_link.header.stamp = rospy.Time(0)
        self.camera_color_optical_frame_base_link.header.frame_id = "left_wrist_3_link"
        self.camera_color_optical_frame_base_link.child_frame_id = "camera_color_optical_frame"
        self.camera_color_optical_frame_base_link.transform.translation.x = -0.103216
        self.camera_color_optical_frame_base_link.transform.translation.y = -0.0348626
        self.camera_color_optical_frame_base_link.transform.translation.z = 0.0216718 
        self.camera_color_optical_frame_base_link.transform.rotation.x = -0.0174581
        self.camera_color_optical_frame_base_link.transform.rotation.y = -0.00310347
        self.camera_color_optical_frame_base_link.transform.rotation.z = -0.383958
        self.camera_color_optical_frame_base_link.transform.rotation.w = 0.92318

        rospy.loginfo(f"\"camera_color_optical_frame\" to \"base_link\" transformation : {self.camera_color_optical_frame_base_link}")
        self.static_broadcaster_camera_color_optical_broadcaster.sendTransform(self.camera_color_optical_frame_base_link)

    def load_config(self):
        with open(self.config_path,"rb") as file:
            self.config = pickle.load(file)
            return self.config

if __name__ == "__main__":
    rospy.init_node("camera_color_optical_frame_static_publisher")
    static_publisher = CameraColorOpticalFrameStaticPublisher()
    rospy.spin()


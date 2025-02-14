#!/usr/bin/env python3

import rospy
import tf
import yaml
import numpy as np
from geometry_msgs.msg import TransformStamped
from moveit_commander import RobotCommander

base_frame = "left_base_link"
EE_frame = "left_wrist_3_link"
world_frame = "world2"

class UR16eCalibration:
    def __init__(self):
        rospy.init_node("ur16e_calibration", anonymous=True)

        # Transform listener and broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.robot = RobotCommander()
        self.points = {}

        rospy.loginfo("Move the robot to each point and enter its index (1, 4, 3)")

    def get_pose(self):
        """ Get the current end-effector pose in 'base' frame """
        try:
            trans, rot = self.tf_listener.lookupTransform(base_frame, EE_frame, rospy.Time(0))
            return np.array(trans), np.array(tf.transformations.euler_from_quaternion(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed!")
            return None, None

    def collect_points(self):
        """ Manually move the robot to each point (1, 4, 3) and record its position """
        for i in [1, 4, 3]:  # Only collect points 1, 4, and 3
            input(f"Move robot to point {i} and press Enter to record...")
            trans, rot = self.get_pose()
            if trans is not None:
                self.points[i] = {"XYZ": trans.tolist(), "RPY": rot.tolist()}
                rospy.loginfo(f"Recorded point {i}: {self.points[i]}")
            else:
                rospy.logwarn("Failed to record point, retry!")

    def compute_transformation(self):
        """ Compute transformation from square (point 4 as origin) to base """
        p4 = np.array(self.points[4]["XYZ"])
        p1 = np.array(self.points[1]["XYZ"])
        p3 = np.array(self.points[3]["XYZ"])

        # X-axis direction (p4 -> p3)
        x_axis = (p3 - p4) / np.linalg.norm(p3 - p4)
        # Y-axis direction (p4 -> p1)
        y_axis = (p1 - p4) / np.linalg.norm(p1 - p4)
        # Z-axis as cross product
        z_axis = np.cross(x_axis, y_axis)

        # Transformation matrix
        R = np.column_stack((x_axis, y_axis, z_axis))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p4  # Origin of the square

        # Compute reverse transformation (square -> base)
        T_inv = np.linalg.inv(T)
        trans_inv = T_inv[:3, 3]
        rot_inv = tf.transformations.euler_from_matrix(T_inv[:3, :3])

        self.square_pose = {"XYZ": p4.tolist(), "RPY": tf.transformations.euler_from_matrix(R)}
        self.square_to_base = {"XYZ": trans_inv.tolist(), "RPY": list(rot_inv)}

        rospy.loginfo(f"Square Pose: {self.square_pose}")
        rospy.loginfo(f"Transformation (Square to Base): {self.square_to_base}")

        # Compute base -> world2 transformation (same as square -> base, but inverted)
        self.base_to_world = {"XYZ": trans_inv.tolist(), "RPY": list(rot_inv)}

    def save_yaml(self, filename="calibration_data.yaml"):
        """ Save data to YAML file """
        data = {
            "vertices": self.points,
            "square_pose": self.square_pose,
            "square_to_base": self.square_to_base,
            "base_to_world": self.base_to_world
        }
        with open(filename, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        rospy.loginfo(f"Calibration data saved to {filename}")

    def publish_tf(self):
        """ Publish base to world2 transformation """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = base_frame
            t.child_frame_id = world_frame
            t.transform.translation.x = self.base_to_world["XYZ"][0]
            t.transform.translation.y = self.base_to_world["XYZ"][1]
            t.transform.translation.z = self.base_to_world["XYZ"][2]
            q = tf.transformations.quaternion_from_euler(*self.base_to_world["RPY"])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(
                (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                (q[0], q[1], q[2], q[3]),
                rospy.Time.now(),
                base_frame,
                world_frame
            )

            rate.sleep()

if __name__ == "__main__":
    calib = UR16eCalibration()
    calib.collect_points()
    calib.compute_transformation()
    calib.save_yaml()

    rospy.loginfo("Starting TF publisher for 'world2' frame...")
    calib.publish_tf()

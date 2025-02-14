#!/usr/bin/env python3

import rospy
import moveit_commander
# from moveit_commander.robot_trajectory import RobotTrajectory

class UR16eMove:
    def __init__(self):
        rospy.init_node("ur16e_move_joint_states", anonymous=True)

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("left_arm")  # Default planning group

        # Define joint states to move to
        self.joint_targets = [
            [-0.447, -2.079, -1.370, -0.998, 1.565, -0.434],  # Joint State 1
            [0.070, -1.828, -1.858, -0.797, 1.436, 0.068],    # Joint State 2
            [0.121, -1.438, -2.482, -0.570, 1.424, 0.118]     # Joint State 3
        ]

    def move_to_joint_state(self, joint_values):
        """ Move the robot to the specified joint state """
        self.group.go(joint_values, wait=True)
        self.group.stop()

    def move_to_all_states(self):
        """ Move to all predefined joint states sequentially """
        for idx, joint_values in enumerate(self.joint_targets, start=1):
            input(f"Press Enter to move to Joint State {idx}...")
            self.move_to_joint_state(joint_values)

        rospy.loginfo("Motion to all joint states completed!")

if __name__ == "__main__":
    ur16e = UR16eMove()
    ur16e.move_to_all_states()

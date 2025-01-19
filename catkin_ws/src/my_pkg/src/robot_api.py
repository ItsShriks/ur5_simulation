#!/usr/bin/env python3

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import sys

class UR5API:
    def __init__(self):
        # Initialize MoveIt Commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_api_node', anonymous=True)
        rospy.loginfo("Waiting for /joint_states...")
        rospy.wait_for_message('/joint_states', JointState)

        # Initialize robot components
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)

    def get_robot_state(self):
        """Returns the current robot state."""
        return self.robot.get_current_state()

    def move_to_joint_position(self, joint_goal):
        """Moves the robot to a specific joint configuration."""
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        return self.group.get_current_joint_values()

    def move_to_cartesian_pose(self, pose_goal):
        """Moves the robot to a specific Cartesian pose."""
        self.group.set_pose_target(pose_goal)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return success

    def plan_cartesian_path(self, waypoints):
        """Plans a Cartesian path."""
        plan, fraction = self.group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # eef_step, jump_threshold
        )
        return plan, fraction

    def execute_plan(self, plan):
        """Executes a planned trajectory."""
        self.group.execute(plan, wait=True)

# Shutdown handler
def shutdown():
    rospy.loginfo("Shutting down UR5 API.")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == "__main__":
    rospy.on_shutdown(shutdown)
    ur5 = UR5API()
    rospy.spin()
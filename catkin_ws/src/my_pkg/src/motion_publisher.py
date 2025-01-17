#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class MotionPublisher:
    def __init__(self):
        rospy.init_node("motion_publisher", anonymous=True)

        # Publisher for joint trajectory
        self.joint_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

        # Robot joint names
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.rate = rospy.Rate(10)  # 10 Hz

    def joint_motion(self, point1, point2, velocity, acceleration):
        """Creates a joint trajectory between two points in joint space."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        # Time to reach the target
        duration = max(np.abs(np.array(point2) - np.array(point1)) / velocity)

        # Trajectory point 1 (start)
        point_start = JointTrajectoryPoint()
        point_start.positions = point1
        point_start.time_from_start = rospy.Duration(0)

        # Trajectory point 2 (end)
        point_end = JointTrajectoryPoint()
        point_end.positions = point2
        point_end.velocities = [velocity] * len(self.joint_names)
        point_end.accelerations = [acceleration] * len(self.joint_names)
        point_end.time_from_start = rospy.Duration(duration)

        # Add points to trajectory
        traj_msg.points = [point_start, point_end]

        # Publish trajectory
        rospy.loginfo("Publishing joint motion...")
        self.joint_pub.publish(traj_msg)
        self.rate.sleep()

    def cartesian_motion(self, pose1, pose2, linear_velocity, linear_acceleration):
        """Creates a linear motion between two Cartesian poses."""
        rospy.loginfo("Cartesian motion requires an inverse kinematics solver.")
        rospy.loginfo(f"Moving from {pose1} to {pose2} at velocity {linear_velocity}.")
        # Implement inverse kinematics here to compute joint angles
        # For now, just log a message
        # (In a full implementation, use a library like KDL or MoveIt to compute IK solutions)
        pass

if __name__ == "__main__":
    try:
        motion_publisher = MotionPublisher()

        # Example points for joint motion
        point1 = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        point2 = [1.0, -1.0, 1.0, 0.5, 0.0, 0.0]
        velocity = 0.5
        acceleration = 0.2

        # Publish joint motion
        motion_publisher.joint_motion(point1, point2, velocity, acceleration)

        # Example poses for Cartesian motion (requires IK solver to implement fully)
        pose1 = [0.4, 0.2, 0.5, 0, 1.57, 0]
        pose2 = [0.5, -0.2, 0.4, 0, 1.57, 0]
        linear_velocity = 0.1
        linear_acceleration = 0.05
        motion_publisher.cartesian_motion(pose1, pose2, linear_velocity, linear_acceleration)

    except rospy.ROSInterruptException:
        pass
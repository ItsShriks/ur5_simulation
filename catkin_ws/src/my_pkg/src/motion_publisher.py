#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class MotionPublisher:
    def __init__(self):
        rospy.init_node("motion_publisher", anonymous=True)

        # Publisher for joint trajectory
        self.joint_pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

        # Robot joint names
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.rate = rospy.Rate(50)  # 10 Hz

    def joint_motion(self, point1, point2, velocity, acceleration):
        """Creates a joint trajectory between two points in joint space."""
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Compute elapsed time
            elapsed = (rospy.Time.now() - start_time).to_sec()

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        # Time to reach the target
        duration = max(np.abs(np.array(point2) - np.array(point1)) / velocity)

        # Trajectory point 1 (start)
        point_start = JointTrajectoryPoint()
        point_start.positions = point1
        point_start.time_from_#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class MotionPublisher:
    def __init__(self):
        rospy.init_node("motion_publisher", anonymous=True)

        # Publisher for joint trajectory
        self.joint_pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

        # Robot joint names
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.rate = rospy.Rate(50)  # 50 Hz

    def joint_motion(self, point1, point2, velocity, acceleration):
        """Creates a joint trajectory between two points in joint space."""
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Compute elapsed time
            elapsed = (rospy.Time.now() - start_time).to_sec()

            # Interpolate between point1 and point2
            interpolated_point = [
                p1 + (p2 - p1) * (elapsed / (np.abs(np.array(point2) - np.array(point1)).max()) or 1)
                for p1, p2 in zip(point1, point2)
            ]

            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names

            # Trajectory point
            point = JointTrajectoryPoint()
            point.positions = interpolated_point
            point.velocities = [velocity] * len(self.joint_names)
            point.accelerations = [acceleration] * len(self.joint_names)
            point.time_from_start = rospy.Duration(elapsed)

            traj_msg.points = [point]

            # Publish trajectory
            rospy.loginfo("Publishing joint motion...")
            self.joint_pub.publish(traj_msg)

            if elapsed >= np.abs(np.array(point2) - np.array(point1)).max() / velocity:
                break  # Stop once the target is reached

            self.rate.sleep()

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
    except rospy.ROSInterruptException:
        passstart = rospy.Duration(0)

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
    except rospy.ROSInterruptException:
        pass
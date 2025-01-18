#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import JointState

class SineWaveJointPublisher:
    def __init__(self):
        rospy.init_node('sine_wave_joint_publisher', anonymous=True)
        
        # Create a publisher for joint states
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Joint names for UR5
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Sine wave parameters
        self.amplitude = rospy.get_param('~amplitude', 1.0)
        self.frequency = rospy.get_param('~frequency', 0.5)

        self.rate = rospy.Rate(50)  # 50 Hz

    def publish_joint_states(self):
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Compute elapsed time
            elapsed = (rospy.Time.now() - start_time).to_sec()

            # Generate sine wave joint positions
            joint_positions = [
                self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed + phase)
                for phase in range(len(self.joint_names))
            ]

            # Create JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = joint_positions

            # Publish the joint states
            self.pub.publish(joint_state_msg)

            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        sine_wave_publisher = SineWaveJointPublisher()
        sine_wave_publisher.publish_joint_states()
    except rospy.ROSInterruptException:
        pass
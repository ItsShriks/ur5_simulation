#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from robot_api import UR5API
from copilot import CodeCoPilot
from geometry_msgs.msg import Pose

def main():
    # Initialize robot API
    api = UR5API()

    # Initialize the code co-pilot
    co_pilot = CodeCoPilot()

    # Sample user input code for co-pilot
    user_code_input = """
def move_robot_to_position(api, position):
    # Move the robot to the target position
    api.move_linear(target_position=position)
"""
    
    # Get code suggestions
    print("Suggested Code:\n", co_pilot.get_code_suggestion(user_code_input))

    # Move to a specific joint position
    joint_goal = [0, -1.57, 1.57, 0, 0, 0]
    api.move_to_joint_position(joint_goal)

    # Move to Cartesian pose
    pose_goal = Pose()
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.2
    pose_goal.position.z = 0.4
    pose_goal.orientation.w = 1.0
    if api.move_to_cartesian_pose(pose_goal):
        print("Pose goal reached!")

if __name__ == "__main__":
    main()

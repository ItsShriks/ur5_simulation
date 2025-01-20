#!/usr/bin/env python3

import rospy
from robot_api import UR5API
from code_suggestor import CodeSuggestor  # Import code suggestion module

def main():
    # Initialize the robot API
    api = UR5API()
    rospy.loginfo("Robot API Initialized")
    
    # Example code context
    code_context = """def move_to_joint_position(joint_goal):
        # Moves the robot arm to the specified joint positions
    """
    
    # Initialize the code suggestor
    suggestor = CodeSuggestor()

    # Get code suggestion from the LLM
    suggestion = suggestor.get_code_suggestion(code_context)
    rospy.loginfo("Code Suggestion:\n%s", suggestion)

    # Optionally, move the robot
    joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot_state = api.move_to_joint_position(joint_goal)
    rospy.loginfo(f"Current robot joint state: {robot_state}")

if __name__ == "__main__":
    main()
# ur5_simulation

# The Task
 
Simulate a UR5 robot on ROS -Gazebo

# Task 1 🟢
1) Set up a simulation environment for a UR5 robot using Gazebo use an open source example for the usage of ROS and [Gazebo](https://github.com/ros-industrial/universal_robot [ROS-Industrial Universal Robots](http://wiki.ros.org/universal_robot) 
Write a custom ROS node to publish desired joint angles to the UR5 robot in Gazebo publish joint angles as function of sine waves on all of the joints write a launch file which starts all nodes and visualizes the results of this task
# Task 2 🟢
Write a library which generates 2 kinds of motion.
The first motion is a joint motion between 2 points in joint space. This motion has the following inputs: point1, point2, joints velocity, and joints acceleration.
The second motion is a linear motion between 2 poses in cartesian space. This motion has the following inputs: pose1, pose2, linear velocity, linear acceleration. 
Publish the output of the motions to your Gazebo node from task 1
Write a launch file which starts all nodes and visualizes the results of this task.
 
# Task 3 🟡
Provide user API for your motion-library in Python: which should have ability to:
- Read robot state
- Move the robot with joint/linear primitive from task 2
 
# Task 4 🔴
Propose solution to use LLM for code suggestion with API provided in task 3 using open-sources LLM models.
Code a simple interface of such the co-pilot.

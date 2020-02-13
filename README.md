# ROS-Projects
# Transformations

In this project we consider a ROS ecosystem, which consists of a robot with a camera mounted on it as well as an object. We defined 4 coordinate frames:

A base coordinate frame called 'base_frame'
A robot coordinate frame called 'robot_frame'
A camera coordinate frame called 'camera_frame'
An object coordinate frame 'object_frame'
The task was to fill in the code for a node called solution.py that published following transforms:

The transform from the 'base_frame' coordinate frame to the 'object_frame' coordinate frame
The transform from the 'base_frame' coordinate frame to the 'robot_frame' coordinate frame
The transform from the 'robot_frame' coordinate frame to the 'camera_frame' coordinate frame

The following transformations were published:

1) The transform from the 'base_frame' coordinate frame to the 'object_frame' coordinate frame consists of a rotation expressed as (roll, pitch, yaw) of (0.64, 0.64, 0.0) followed by a translation of 1.5m along the resulting x-axis and 0.8m along the resulting y-axis. 
2) The transform from the 'base_frame' coordinate frame to the 'robot_frame' coordinate frame consists of a rotation around the y-axis by 1.5 radians followed by a translation along the resulting z-axis of -2.0m. 
3) The transform from the 'robot_frame' coordinate frame to the 'camera_frame' coordinate frame must be defined as follows:
    a) The translation component of this transform is (0.3, 0.0, 0.3)
    b) The rotation component of this transform must be set such that the camera is pointing directly at the object. In other words, the x-axis of the 'camera_frame' coordinate frame must be pointing directly at the origin of the 'object_frame' coordinate frame. 

# Forward Kinematics

The script computes forward kinematics for industrial robots KUKA lwr and UR5.
Every time a new set of joint values are received, the node uses this information, along with the URDF, and computes the transform from the robot "root" (first link) to each link in the chain. All of these transforms are then published to TF.

# Inverse Kinematics

The script is use to get  Cartesian Control and Numerical Inverse kinematics of industrial robots KUKA lwr and UR5. This project is an implementation of a complete algorithm for Cartesian end-effector translation control with a secondary objective. The file ccik.py contains primarily three functions get_cartesian_command, get_ik_command and get_jacobian.

# Motion Planning

Implementation of the RRT algorithm for sampling-based motion planning. The motion planning algorithm is used along with MoveIt! software framework to help with some needed auxiliary calls. The function motion_planning is a callback for the /motion_planning_goal topic of the data type "geometry_msgs/Transform". Each time the callback is invoked, the robot plans and executes an appropriate joint trajectory using an RRT planner.

# State Estimation

The script implements Extended Kalman Filter for state estimation of a mobile robot; the state consists of the robot's 2D position and orientation. In addition to publishing the commanded velocities, the robot can also localize itself with respect to a number of landmarks in the scene. Every time a landmark is within range of the robot's sensors, the robot will record its distance to the landmark, as well as the bearing of the landmark with respect to the robot's orientation.

# Data Prediction for Grasp Type

This project uses data pertaining to human grasping. With a subject grasping a sequence of objects, we collected data containing their hand joint angles (as measured by a Cyberglove), forearm EMG (collected by a Myo armband) and their true labels. 
The node analysis.py does the following predictions:
1) If the object label is missing, predict it based on glove data (if present) or EMG data (if glove data is not present).
2) If the glove data is missing, predict it based on EMG data.
3) If the glove data is present but the low-dimensional glove data is missing, fill in the low-dimensional glove data by projecting the incoming full-dimensional glove data into a 2D linear subspace that approximates the full-dimensional space as well as possible.
4) If you only receive low-dimensional glove data, fill in the high-dimensional glove data (msg.glove) field with an inverse projection.

# Training Neural Network for forward dynamics

The script implements learning forward dynamics of a 3 link robot using a neural network. There is a "real robot" based on an analytical model and the task was to created a forward dynamics model ("fake robot") trained by the "real robot".
The ROS node fake_robot.py found in the robot_sim package:
1) Creates a dataset that contains features(current state and action) and labels(state at the next time step).
2) Trains a fake robot with the dataset from the real robot with a total time of 10 minutes to collect and train data.
3) Creates a ROS service that responds to the request (torque commands) from a service client from executive node with the estimated state at the next time step.

# Cartpole balancing using Reinforcement Learning

This script is an implementation of a Deep Q-learning algorithm on a cartpole system. A ROS node was created that implements a learning algorithm to control this robot to balance the pole.


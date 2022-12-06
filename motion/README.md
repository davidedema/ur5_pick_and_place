# Kinematics
This folder contains a ros package containing all the motion planning and kinematics code for the robot. In the include folder, there are the header files for using kinematics funtions:
- Forward kinematic
- Inverse kinematic
- Jacobian matrix

All the functions are implemented in the header file `kinematics.h`, all it's documented with doxygen.

In the src folder there are our motion planning algorithms:
- translated testUR57 has to be fixed in some points
- going to translate testUR58
  
## Forward Kinematic
The forward kinematic is used to calculate the position of the end effector of the robot. The function is called with the current joint angles. The function returns the position and orientation of the end effector in the base frame.

## Inverse Kinematic
The inverse kinematic is used to calculate the joint angles for a given position and orientation of the end effector. The function is called with the position and orientation of the end effector in the base frame. The function returns 8 possible joint angles configurations.

## Jacobian Matrix
The jacobian matrix is used to calculate the velocity of the end effector. The function is called with the current joint angles. The function returns the jacobian matrix.


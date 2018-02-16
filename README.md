# COMPGX01 - Robotic Systems Engineering Coursework 1

## Question 7
A ROS service server has been written to convert different representations. The instructions are as followed:
  * Launch this server using `rosrun coursework_1 coursework_1_node`
    * The server will be up, and ready for any ROS service client to request for a response.

Alternatively, three seperate nodes has been written for each part of question 7.
  * Launch part(a) using `roslaunch coursework_1 quat_to_Euler.launch`
  * Launch part(b) using `roslaunch coursework_1 quat_to_Axis.launch`
  * Launch part(c) using `roslaunch coursework_1 rotation_to_Quat.launch`
    * You can pass in the arguments of each launch file with the respective parameter values.

## Question 8(d)
A ROS node has been created to retrieve DH parameters from the published topics by youbot. The respective TF are
broadcasted after obtaining the DH parameters.
  * Launch using `roslaunch coursework_1 forwardKinematic.launch`
    * Remember to launch the youbot simulator by `roslaunch youbot_simulator youbot_sim.launch`
  * The DH parameters are obtained by subscribing to `/gazebo/link_staes` and `/joint_states` topics.
    * Standard DH convention is used, only rotation and translation around z-axis first before x-axis.
    * the frames orientation are a little different from the original youbot due to the DH convention, since ROS
	  is able to rotate and translate around 3 axis.

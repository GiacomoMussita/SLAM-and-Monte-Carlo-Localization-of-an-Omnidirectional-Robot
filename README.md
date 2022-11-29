# SLAM and Monte Carlo Localization of an Omnidirectional Robot

1. PACKAGE INSTRUCTIONS
2. FILE DESCRIPTION
3. ROS PARAMETERS

### 1. PACKAGE INSTRUCTIONS

Copy the `project` folder in the `src` folder of the ROS workspace. Build the package in the ROS workspace:
	
`catkin_make`

Note that the name of the package is `project`, so this name must be specified to run any node of the package from terminal.
<br />Run the package from terminal directly through a launch file (see below for the description of the file):

`roslaunch project project_launch.launch`

Open the configuration file `rvizconfig.rviz` in rviz (see below for the description of the file).
<br />Play one of the bag in their directory (it is suggested to play the bag in loop):

`rosbag play -l bag_i.bag` 

rviz will visualize the pose from OptiTrack (yellow arrow) and the odometry (red arrow).
<br />The computational graph (nodes and topics) can be analyze through:

`rqt_graph`

Other useful commands to draw the tree of transformation frames, plot and compare signals from different topics, or open the dynamic reconfigure window are:

`rosrun rqt_tf_tree rqt_tf_tree`
<br />`rosrun rqt_plot rqt_plot`
<br />`rosrun plotjuggler plotjuggler`
<br />`rosrun rqt_reconfigure rqt_reconfigure`

To reset odometry when the nodes are running:

`rosservice call reset new_x new_y new_theta`

### 2. FILE DESCRIPTION

**************

`Kinematics.cpp`, `main_kinematics.cpp`: c++ files that implement the node `kinematics`. This node computes the forward kinematics (linear and angular velocities) starting from the control inputs (rotations of the wheels). It subscribes to topic `wheels_states` published from the bag file, it reads the encoders ticks count, and it calculates the velocities using these formulas:

<br />Linear speed along x [m/s], vx = r * [u1 + u2 + u3 + u4] / 4
<br />Linear speed along y [m/s], vy = r * [-u1 + u2 + u3 - u4] / 4
<br />Angular speed around z axis [rad/s], thetadot = r * [-u1 + u2 - u3 + u4] / (4 * (l + w))

<br />r : wheel radius [m]
<br />u_i : wheel speed i, i = 1,2,3,4 (computed from the encoders ticks) [rad/s]
<br />l : wheel position along x [m]
<br />w : wheel position along y [m]

<br />u_i = 2 * pi * dtick_i / (dT * CPR * N)

<br />dtick_i : encoder ticks increment [ticks/s]
<br />dT : time interval [s]
<br />CPR : counts per revolution [ticks/rev]
<br />N : gear ratio  

Then, the node publishes these velocities as a `geometry_msgs/TwistStamped` message on topic `cmd_vel`.
In this node, as in the following ones (except for `broadcaster`), a condition to reset the node variables when the bags restart has been implemented. In this way, the bags can be played in loop without undesired behaviour, like negative delta times, meaningless velocities or incremental integration. The code uses the time stamp: when the new time is smaller than the current time, it means that the bag has restarted. However, note that if the bag is changed while the nodes are still running, this condition is not guaranteed to work.

A dynamic server is implemented to allow changing the robot parameters rapidly through a dynamic reconfigure window, without needing to restart the node. This feature is really useful to perform manual calibration of the parameters (see later).

**************

`Odometry.cpp`, `main_odometry.cpp`: c++ files that implement the node `odometry`. This node performs Euler and Runge-Kutta velocity integrations to obtain the pose (x,y,theta). 

Euler:
<br />Position along x [m]: x_k+1 = x_k + dT * vx * cos(theta_k) - dT * vy * sin(theta_k)
<br />Position along y [m]: y_k+1 = y_k + dT * vx * sin(theta_k) + dT * vy * cos(theta_k)
<br />Angular position [rad]: theta_k+1 = theta_k + dT * thetadot

Runge_Kutta:
<br />Position along x [m]: x_k+1 = x_k + dT * vx * cos(theta_k + 0.5 * thetadot * dT) - dT * vy * sin(theta_k + 0.5 * thetadot * dT)
<br />Position along y [m]: y_k+1 = y_k + dT * vx * sin(theta_k + 0.5 * thetadot * dT) + dT * vy * cos(theta_k + 0.5 * thetadot * dT)
<br />Angular position [rad]: theta_k+1 = theta_k + dT * thetadot

There is no huge difference in results between the two methods, especially because the rotation is equivalent for both methods, therefore, errors on rotation propagate anyway. Moreover, in linear motion the two methods are exactly the same. However, Runge-Kutta odometry performs slightly  better in free motions and odometry is more precise.
<br />Once computed odometry, the node publishes the pose as a `nav_msgs/Odometry` message on topic `odom`.
The rotation in this message is represented through quaternions (4 numbers representation). Since the robot performs only a planar motion, the only component of rotation different from zero is the yaw (rotation around z axis), thus, it is fairly simple to compute the quaternions from theta:

<br />qw = cos(theta/2)
<br />qx = 0
<br />qy = 0
<br />qz = sin(theta/2)

A server is implemented to reset the odometry, calling the method `resetOdometry`.
A dynamic server is implemented to switch from one integration method to the other. 

**************

`Forwardkin.cpp`, `main_controlspeed.cpp`: c++ files that implement the node `controlspeed`. This node takes the speed reference from topic `cmd_vel` and calculates the command speed, publishing them on topic `wheels_rpm` as a message `project::CustomRpm`. Using the plotjuggler tool (write on terminal `rosrun plotjuggler plotjuggler`) it is possible to assess the precision of the result, comparing it with the true command velocities of the encoders in topic `wheels_state`. 

**************

`TfBroad.cpp`, `main_broadcaster.cpp`: c++ files that implement the node `broadcaster`. This node takes the pose from topic `odom` and computes the transformation between the reference frame of the robot, named `base_link`, and the reference frame with respect to which the odometry is computed, named `odom`. This can be easily done by considering the translation between the two frames equal to the position (x,y) of the robot, and the rotation between the two frames equal to the quaternions describing the robot orientation (ROS will automatically build the homogeneous transformation matrix). Then, the node publishes the transformation.

**************

`include` directory: this contains all the header files of the c++ classes implemented in the project.

**************

`mode.cfg`: configuration file to switch from Euler to Runge-Kutta using a dynamic server. Running `rqt_reconfigure` from terminal it is possibile to switch mode at any time. 

**************

`param.cfg`: configuration file to change the parameters (radius, CPR, length, width) of the robot during calibration. An `rqt_reconfigure` window was used to ease the calibration process. 

**************

`CustomRpm.msg`: definition of the custom message for the wheel speeds.

**************

`reset.srv`: definition of request and response type of the service that implements the reset of the odometry. Note however that after resetting the odometry to any pose, this won't match anymore the one from OptiTrack in rviz, because the frames with respect to which the odometry and the OptiTrack poses are taken are always aligned.

**************

`project_launch.launch`: launch file that starts all the nodes implemented (`kinematics`, `odometry`, `controlspeed`, `broadcaster`). Within the launch it is specified also a publisher of the static transformation between child frame `odom` (with respect to which the odometry is described) and parent frame `world` (with respect to which OptiTrack pose is described). This transformation is an identity transformation: origins and axes of the two frames are aligned. The last node specified is the one that opens rviz. In the launch file are initialize also the ROS parameters.

**************

`CMakeLists.txt`: inside this file is specified the name of the package. 

**************

`package.xml`

**************

`rvizconfig.rviz`: rviz file with the configuration of the axes (`world` frame and `odom` frame) and of the poses. The poses are described with two arrows, the red one is the odometry, the yellow one is from OptiTrack.

### 3. ROS PARAMETERS

<br />Wheel radius (`radius`): 0.077 [m] 
<br />Wheel position along x (`length`): 0.204 [m]
<br />Wheel position along y (`width`): 0.174 [m]
<br />Counts per revolution of the encoder (`CPR`): 40 [count/rev]
<br />Gear ratio (`gear_ratio`): 5
<br />Pi: 3.14159 (value of pigreco common for all nodes to reduce errors)
<br />x0: 0.0
<br />y0: 0.0 
<br />theta0: 0.0



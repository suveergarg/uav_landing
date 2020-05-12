# uav_landing

## Overview

The project creates an Unmanned Autonomous Vehicle (UAV) capable of landing on stationary and moving target. The UAV is maneuvered through a controller and has the ability to optimally track, detect and land on the target. 

![simulated](https://github.com/suveergarg/uav_landing/blob/master/images/simulated-world.png | width=100)

**Keywords:** moving target, stationary target, landing

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [hector_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor "hector_quadrotor") (quadcopter used for the project)
- [hector_gazebo](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo "hector_gazebo") (gazebo support for hector quadrotor)
- [tf_velocity_estimator](https://github.com/gstavrinos/tf_velocity_estimator "tf_velocity_estimator") (A ROS package that helps estimate the veolocity of a TF frame)
- [ar_helipad](https://github.com/gstavrinos/ar_helipad "ar_helipad") (A ROS package that is used to detect helipads (landing zones) that have one or two (bundled) AR tags)
- [ar_track_alvar](https://github.com/ros-perception/ar_track_alvar "ar_track_alvar")



#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/suveergarg/uav_landing
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


## Usage

Run the main node with

	roslaunch uav_landing main.launch

## Nodes
### exploration_node

#### Published Topics

* **`/action/pose/goal`** ([hector_uav_msgs/PoseActionGoal])
	Sends the target position to acheive for the platform

* **`/cmd_vel  `** ([geometry_msgs/Twist ])

### moving_helipad

#### Subscribed Topics

* **`/gazebo/model_states`** ([gazebo_msgs  /ModelStates])
	Gets the ground truth value for the position of platform


#### Published Topics

* **`/gazebo/set_model_state `** ([gazebo_msgs /ModelState])
	Sets the ground truth value for the position of platform

### estimate_pad_velocity

#### Subscribed Topics

* **`/tf`** ([tf2_msgs/TFMessage])


#### Published Topics

* **`/pad_velocity `** ([tf_velocity_estimator/PosesAndVelocities])
	Send the platform velocity and position


### pid_tuner

#### Subscribed Topics

* **`/ground_truth/state`** ([nav_msgs/Odometry])


#### Published Topics

* **`/pid_tuner  `** ([std_msgs/Bool])
	Send the platform velocity and position

### controller1


#### Subscribed Topics

* **`/ground_truth/state`** ([nav_msgs /Odometry])

* **`/pad_velocity `** ([tf_velocity_estimator /PosesAndVelocities ])
	Gets the estimated landing platform velocity


#### Published Topics

* **`/command/pose `** ([geometry_msgs/PoseStamped  ])

* **`/command/twist `** ([geometry_msgs /TwistStamped   ])
	Send commands to be executed for the quadrotor

* **`/cmd_vel`** ([geometry_msgs  /Twist ])

* **`/pid_tuner `** ([std_msgs /Bool   ])
	Informs pid when to start/stop recording the trajectory being generated


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html

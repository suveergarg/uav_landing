#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 20:27:59 2020

@author: gsuveer
"""


from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
from tf_velocity_estimator.msg import PosesAndVelocities
from tf_velocity_estimator.msg import Velocity
from geometry_msgs.msg import PoseStamped
import rospy


# Define the trajectory starting state:
pos0 = [0, 0, 2] #position
vel0 = [0, 0, 0] #velocity
acc0 = [0, 0, 0] #acceleration

# Define the goal state:
posf = [1, 0, 1]  # position
velf = [0, 0, 1]  # velocity
accf = [0, 9.81, 0]  # acceleration

# Define the duration:
Tf = 1

# Define the input limits:
fmin = 5  #[m/s**2]
fmax = 25 #[m/s**2]
wmax = 20 #[rad/s]
minTimeSec = 0.02 #[s]

# Define how gravity lies:
gravity = [0,0,-9.81]
 
trajnav_msgs/Odometry = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
traj.set_goal_position(posf)
traj.set_goal_velocity(velf)
traj.set_goal_acceleration(accf)

# Note: if you'd like to leave some states free, there are two options to 
# encode this. As exmample, we will be leaving the velocity in `x` (axis 0)
# free:
#
# Option 1: 
# traj.set_goal_velocity_in_axis(1,velf_y);
# traj.set_goal_velocity_in_axis(2,velf_z);
# 
# Option 2:
# traj.set_goal_velocity([None, velf_y, velf_z])
 
# Run the algorithm, and generate the trajectory.
traj.generate(Tf)

# Test input feasibility
inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)

# Test whether we fly into the floor
floorPoint  = [0,0,0]  # a point on the floor
floorNormal = [0,0,1]  # we want to be in this direction of the point (upwards)
positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)


goal_msg = PoseStamped()
def callback(msg):
    global goal_msg
    latest_poses = msg.latest_poses
    #print(type(msg.latest_poses))
    avg_x, avg_y, avg_z =0,0,0
    delta_t = 0.1
	
    for i in range(len(latest_poses)):
        avg_x = avg_x + latest_poses[i].pose.position.x 
        avg_y = avg_y + latest_poses[i].pose.position.y
        avg_z = avg_z + latest_poses[i].pose.position.z

    #delta_t = goal_msg.header.stamp.secs  - latest_poses[i].header.stamp.secs
    goal_msg.header.stamp  = latest_poses[i].header.stamp    
    goal_msg.header.frame_id =  'world'
    goal_msg.pose.position.x = avg_x/len(latest_poses) + avg_x*delta_t/len(latest_poses)
    goal_msg.pose.position.y = avg_y/len(latest_poses) + avg_y*delta_t/len(latest_poses)
    goal_msg.pose.position.z = avg_z/len(latest_poses) + avg_z*delta_t/len(latest_poses) + 5
    

rospy.init_node('controller_node')
sub = rospy.Subscriber('/pad_velocity', PosesAndVelocities,callback, queue_size = 10)
pub = rospy.Publisher('/command/pose',PoseStamped, queue_size = 10)
while not rospy.is_shutdown():
    rospy.spin()

  
    


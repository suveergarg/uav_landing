#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Sun May 10 18:36:06 2020

@author: gsuveer

"""
from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from tf_velocity_estimator.msg import PosesAndVelocities
from tf_velocity_estimator.msg import Velocity
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from quadrocoptertrajectory import SingleAxisTrajectory
from std_msgs.msg import Bool
import subprocess
from plot import pidTuner, timeVsDist

def modelStatesCallback(msg):
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub, waypoints_index,waypoints
    index_of_interest = -1

    if index_of_interest >= 0:
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = msg.pose[index_of_interest]
        twist = Twist()



rospy.init_node(' count_node ')

model_state_sub = rospy.Subscriber("gazebo/model_states", ModelStates, modelStatesCallback)

    


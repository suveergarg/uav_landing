#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May  8 17:46:47 2020

@author: gsuveer
"""

from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
from tf_velocity_estimator.msg import PosesAndVelocities
from tf_velocity_estimator.msg import Velocity
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from quadrocoptertrajectory import SingleAxisTrajectory
import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from std_msgs.msg import Bool

rospy.init_node('PID_Tuner')

Record = False
flag   = False

numPlotPoints = 100000
time = np.zeros(numPlotPoints)
position = np.zeros([numPlotPoints, 3])
velocity = np.zeros([numPlotPoints, 3])

i=0

def callback(msg):
    
    global position,velocity,i 
    
    if(Record):
        
        position[i,0] = msg.pose.pose.position.x
        position[i,1] = msg.pose.pose.position.y
        position[i,2] = msg.pose.pose.position.z
        
        velocity[i,0] = msg.twist.twist.linear.x
        velocity[i,1] = msg.twist.twist.linear.y
        velocity[i,2] = msg.twist.twist.linear.z
    
        i = i+1
    

def callback_flag(msg):
    if (msg.data == True):
        Record = True
    if (msg.data == False):
        flag = True
        

odometry_subscriber = rospy.Subscriber('/ground_truth/state' , Odometry, callback)
flag_subscriber     = rospy.Subscriber('/pid_tuner',  Bool, callback_flag)

import matplotlib.pyplot as plt

while not rospy.is_shutdown():
    
    if(flag):
        flag = False
        figStates, axes = plt.subplots(3,1,sharex=True)
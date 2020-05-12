'''
Node for recording the response of the PID controller

'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May  8 17:46:47 2020

@author: gsuveer
"""

from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
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
    
    '''
    
    Callback funtion for Recording Ground Truth Values of the Quadrotor from Gazebo.
    
    '''
    
    global position, velocity, i, time, Record, flag 
    
    if(Record):
        
        position[i,0] = msg.pose.pose.position.x
        position[i,1] = msg.pose.pose.position.y
        position[i,2] = msg.pose.pose.position.z
        
        velocity[i,0] = msg.twist.twist.linear.x
        velocity[i,1] = msg.twist.twist.linear.y
        velocity[i,2] = msg.twist.twist.linear.z
        time[i] = i
        
        i = i+1
    
    
    if(flag):
        print("Generating plots")
        flag = False
        Record = False
        idx = time>0
        time         = time[idx]
        position     = position[idx, :] 
        velocity     = velocity[idx, :] 
        
        plt.figure()
        
        plt.subplot(2,1,1)
        plt.plot(time, position[:,0], 'r', time, position[:,1], 'g', time, position[:,2], 'b')
        
        plt.subplot(2,1,2)
        plt.plot(time, velocity[:,0], 'r', time, velocity[:,1], 'g', time, velocity[:,2], 'b')
        
        time = np.zeros(numPlotPoints)
        position = np.zeros([numPlotPoints, 3])
        velocity = np.zeros([numPlotPoints, 3]) 
        
        plt.show()

def callback_flag(msg):
    
    '''
    
    Callback function to decide when to start / stop recording after the trajectory has been generated
    
    '''
    
    global Record, flag
    if (msg.data == True):
        Record = True
    if (msg.data == False):
        flag = True
    print("Received Message ", msg)
    
        
odometry_subscriber = rospy.Subscriber('/ground_truth/state' , Odometry, callback)
flag_subscriber     = rospy.Subscriber('/pid_tuner',  Bool, callback_flag)

import matplotlib.pyplot as plt

while (True):
    pass
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
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from quadrocoptertrajectory import SingleAxisTrajectory
from std_msgs.msg import Bool

rospy.init_node('controller_minsnap_node')

plot_controller = 0
landing_mode_on = False
last_goal = None
# Define the trajectory starting state:
pos0 = [0, 0, 2] #position
vel0 = [0, 0, 0] #velocity
acc0 = [0, 9.81, 0] #acceleration

# Define the goal state:
posf = [1, 0, 1]  # position
velf = [0, 0, 1]  # velocity
accf = [0, 9.81, 0]  # acceleration

# Define the duration:
Tf = 5

# Define the input limits:
fmin = 5  #[m/s**2]
fmax = 25 #[m/s**2]
wmax = 20 #[rad/s]
minTimeSec = 0.02 #[s]

# Define how gravity lies:
gravity = [0,0,-9.81]

t_init = rospy.get_time()

traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)
feasible = False
counter  = 0

def gen_traj(pos0, vel0, acc0, posf, velf, accf):
    
    global traj
    
    traj._axis = [SingleAxisTrajectory(pos0[i],vel0[i],acc0[i]) for i in range(3)]
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
    print("Trajectory Generated")
    # Test input feasibiliCTRL-C to quit


    inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)
    
    # Test whether we fly into the floor
    floorPoint  = [0,0,0]  # a point on the floor
    floorNormal = [0,0,1]  # we want to be in this direction of the point (upwards)
    positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)
    if(inputsFeasible == 0 and positionFeasible == 0):
    # IF Feasiable, return trajectory.
        feasible = True
    else:
        feasible = False


def callback_pad(msg):
    
    global pos0, vel0, acc0, posf, velf, accf, t_init, counter, landing_mode_on, last_goal
    latest_poses    = msg.latest_poses
    latest_velocity = msg.latest_velocities 
    #print(type(msg.latest_poses))
    avg_x, avg_y, avg_z, avg_vx, avg_vy, avg_vz =0,0,0,0,0,0
    delta_t = 0
	
    for i in range(len(latest_poses)):
        avg_x = avg_x + latest_poses[i].pose.position.x 
        avg_y = avg_y + latest_poses[i].pose.position.y
        avg_z = avg_z + latest_poses[i].pose.position.z
        avg_vx = avg_vx + latest_velocity[i].vx
        avg_vy = avg_vy + latest_velocity[i].vy
        avg_vz = avg_vz + latest_velocity[i].vz

    #delta_t = goal_msg.header.stamp.secs  - latest_poses[i].header.stamp.secs
    
    avg_vx = avg_vx/len(latest_poses)
    avg_vy = avg_vy/len(latest_poses)
    avg_vz = avg_vz/len(latest_poses)
    
    x = avg_x/len(latest_poses) + avg_vx*delta_t
    y = avg_y/len(latest_poses) + avg_vy*delta_t
    z = avg_z/len(latest_poses) + avg_vz*delta_t

    if not landing_mode_on:
        z = 5
    else:
        print('last goal is : ', last_goal)
        z = 0
        x = last_goal[0][0]
        y = last_goal[0][1]
        avg_vx, avg_y, avg_z = last_goal[1]

    # print(posf)
    posf = [x, y, z]
    velf = [avg_vx, avg_y, avg_z]
    last_goal = [posf, velf, accf]

    
    
    
    #Can add a reasonsable condition for replanning trajectory
    
    # t_init = rospy.get_time()
    # counter = counter + 1
    # if(counter ==  10):
    #     print("generating new trajectory")
    #     gen_traj(pos0, vel0, acc0, posf, velf, accf)
    #     counter = 0
    
    
def callback_quad(msg):
    global pos0,vel0, landing_mode_on
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    z=msg.pose.pose.position.z    
    # print('pose information of quadrotor: ', x, y, z)
    pos0 = [x,y,z]
    vx =msg.twist.twist.linear.x
    vy =msg.twist.twist.linear.y
    vz =msg.twist.twist.linear.z
    vel0 = [vx, vy, vz]

    if(z < 5):
        landing_mode_on = True
        print('landing mode on')


#To be replaced by odom later on
sub_quadstate =  rospy.Subscriber('/ground_truth/state', Odometry, callback_quad, queue_size = 10)
sub_padstate  = rospy.Subscriber('/pad_velocity', PosesAndVelocities, callback_pad, queue_size = 10)

pub_pos = rospy.Publisher('/command/pose',  PoseStamped, queue_size = 10)
pub_vel = rospy.Publisher('/command/twist', TwistStamped, queue_size = 10)
pub_vel_msg = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

pos_goal_msg = PoseStamped()
#vel_goal_msg = TwistStamped()
vel_goal_msg = Twist()

msg = rospy.wait_for_message("/pad_velocity", PosesAndVelocities , timeout=5)

#callback_pad(msg)
# t_init = rospy.get_time()    
gen_traj(pos0, vel0, acc0, posf, velf, accf)
flag_publisher = rospy.Publisher('/pid_tuner',  Bool, queue_size = 10)
flag=Bool()
flag.data = True
flag_publisher.publish(flag)


'''
Plotting Code
'''

import numpy as np
numPlotPoints = 100000
time = np.zeros(numPlotPoints)
position = np.zeros([numPlotPoints, 3])
velocity = np.zeros([numPlotPoints, 3])
acceleration = np.zeros([numPlotPoints, 3])
thrust = np.zeros([numPlotPoints, 1])
ratesMagn = np.zeros([numPlotPoints,1])
i = 0

while not rospy.is_shutdown():
    
    #Generate Trajectories
    t = rospy.get_time()
    # print('t and t_init ', t, t_init)
    if(t-t_init < Tf):
        try:
    
            velocity_ = traj.get_velocity(t - t_init)
            vel_goal_msg.linear.x = velocity_[0]
            vel_goal_msg.linear.y = velocity_[1]
            vel_goal_msg.linear.z = velocity_[2]
            
            position_ = traj.get_position(t- t_init)
            pos_goal_msg.header.frame_id = 'world'
            pos_goal_msg.pose.position.x = position_[0]
            pos_goal_msg.pose.position.y = position_[1]
            pos_goal_msg.pose.position.z = position_[2]
            
            # print(pos_goal_msg)
            #pub_vel_msg.publish(vel_goal_msg)

            #Plotting Code
            time[i]        = t - t_init
            position[i, :] = traj.get_position(t - t_init)
            velocity[i, :] = traj.get_velocity(t - t_init)
            acceleration[i, :] = traj.get_acceleration(t - t_init)
            thrust[i] = traj.get_thrust(t - t_init)
            ratesMagn[i] = np.linalg.norm(traj.get_body_rates(t - t_init))
            i = i + 1
             
            
            pub_pos.publish(pos_goal_msg)

        except:
            print("Error")
            continue
        
    else:
        # if(landing_mode_on):
        #     print('landing mode on, goal to achieve')
        #     print('\n---------------\n', posf, velf, accf)
        #     gen_traj(pos0, vel0, acc0, posf, velf, accf)
        #     t_init = rospy.get_time()

        # else:
        i = 0
        ('\nhere---------------\n', posf, velf, accf)
        gen_traj(pos0, vel0, acc0, posf, velf, accf)
        last_goal = [posf, velf, accf]
        t_init = rospy.get_time()
       # break



try:   
    # Plotting Code
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
     

    # Removing Zero Index

    idx = time>0
    time         = time[idx]
    position     = position[idx, :] 
    velocity     = velocity[idx, :] 
    acceleration = acceleration[idx, :]  
    thrust       = thrust[idx] 
    ratesMagn    =  ratesMagn[idx]  


    figStates, axes = plt.subplots(3,1,sharex=True)
    gs = gridspec.GridSpec(6, 2)
    axPos = plt.subplot(gs[0:2, 0])
    axVel = plt.subplot(gs[2:4, 0])
    axAcc = plt.subplot(gs[4:6, 0])

    for ax,yvals in zip([axPos, axVel, axAcc], [position,velocity,acceleration]):
        cols = ['r','g','b']
        labs = ['x','y','z']
        for i in range(3):
            ax.plot(time,yvals[:,i],cols[i],label=labs[i])

    axPos.set_ylabel('Pos [m]')
    axVel.set_ylabel('Vel [m/s]')
    axAcc.set_ylabel('Acc [m/s^2]')
    axAcc.set_xlabel('Time [s]')
    axPos.legend()
    axPos.set_title('States')

    infeasibleAreaColour = [1,0.5,0.5]
    axThrust = plt.subplot(gs[0:3, 1])
    axOmega  = plt.subplot(gs[3:6, 1])
    axThrust.plot(time,thrust,'k', label='command')
    axThrust.plot([0,Tf],[fmin,fmin],'r--', label='fmin')
    axThrust.fill_between([0,Tf],[fmin,fmin],-1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.fill_between([0,Tf],[fmax,fmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axThrust.plot([0,Tf],[fmax,fmax],'r-.', label='fmax')

    axThrust.set_ylabel('Thrust [m/s^2]')
    axThrust.legend()

    axOmega.plot(time, ratesMagn,'k',label='command magnitude')
    axOmega.plot([0,Tf],[wmax,wmax],'r--', label='wmax')
    axOmega.fill_between([0,Tf],[wmax,wmax], 1000,facecolor=infeasibleAreaColour, color=infeasibleAreaColour)
    axOmega.set_xlabel('Time [s]')
    axOmega.set_ylabel('Body rates [rad/s]')
    axOmega.legend()

    axThrust.set_title('Inputs')

    axThrust.set_ylim([min(fmin-1,min(thrust)), max(fmax+1,max(thrust))])
    axOmega.set_ylim([0, max(wmax+1,max(ratesMagn))])

    plt.show()

except:
    print('caught error in plotting logic')

    
            
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

rospy.init_node('controller_minsnap_node')

# Define the trajectory starting state:
pos0 = [0, 0, 2] #position
vel0 = [0, 0, 0] #velocity
acc0 = [0, 9.81, 0] #acceleration

# Define the goal state:
posf = [1, 0, 1]  # position
velf = [0, 0, 1]  # velocity
accf = [0, 9.81, 0]  # acceleration

# Define the duration:
Tf = 3

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
    
    global pos0, vel0, acc0, posf, velf, accf, t_init, counter
    latest_poses    = msg.latest_poses
    latest_velocity = msg.latest_velocities 
    #print(type(msg.latest_poses))
    avg_x, avg_y, avg_z, avg_vx, avg_vy, avg_vz =0,0,0,0,0,0
    delta_t = 0.2
	
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
    #z = avg_z/len(latest_poses) + avg_vz*delta_t
    
    z = 5
    
    posf = [x, y, z]
    velf = [avg_vx, avg_y, avg_z]
    
    
    t_init = rospy.get_time()
    #Can add a reasonsable condition for replanning trajectory
    counter = counter + 1
    if(counter > 0 and pos0[2] > 4): #Based on the assumption that trajectory generation kicks in when quadrotor altitude > 4
        gen_traj(pos0, vel0, acc0, posf, velf, accf)
        counter = 0
    elif(pos0[2] <= 4):
        print('landing mode on')
    
def callback_quad(msg):
    global pos0,vel0
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    z=msg.pose.pose.position.z    
    # print('pose information of quadrotor: ', x, y, z)
    pos0 = [x,y,z]
    vx =msg.twist.twist.linear.x
    vy =msg.twist.twist.linear.y
    vz =msg.twist.twist.linear.z
    vel0 = [vx, vy, vz]
 


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

while not rospy.is_shutdown():
    #Generate Trajectories    
    t = rospy.get_time()
    if (t-t_init < Tf):
        try:
            
            velocity = traj.get_velocity(t - t_init)
            '''
            vel_goal_msg.twist.linear.x = velocity[0]
            vel_goal_msg.twist.linear.y = velocity[1]
            vel_goal_msg.twist.linear.z = velocity[2]
            vel_goal_msg.header.frame_id = 'world'
            '''
            velocity = traj.get_velocity(t - t_init)
            vel_goal_msg.linear.x = velocity[0]
            vel_goal_msg.linear.y = velocity[1]
            vel_goal_msg.linear.z = velocity[2]
            
            position = traj.get_position(t- t_init)
            pos_goal_msg.header.frame_id = 'world'
            pos_goal_msg.pose.position.x = position[0]
            pos_goal_msg.pose.position.y = position[1]
            pos_goal_msg.pose.position.z = position[2]
            
            #print(pos_goal_msg)
            #pub_vel_msg.publish(vel_goal_msg)
            
            pub_pos.publish(pos_goal_msg)
            # print("sending command  ", t - t_init)
        except:
            print("Error")
            continue
            
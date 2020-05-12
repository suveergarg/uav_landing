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
import subprocess
from plot import pidTuner, timeVsDist, trajectory3D
'''

Node for dynamic computation of minimum snap trajectory and sending position goals to PI position controller

'''
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
Tf = 1

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

landing_mode      = False
landing_executed  = False
landing_threshold = 5

timestampsList = []
distanceFromGround = []
xline, yline = [], []

def point_sqaure_collision(point, rect):
    '''
    
    Check for landing on the pad 
    
    Returns: Bool
    
    '''
    if(abs(point[2] - rect[2]) > 0.3):
        return False
    # print(point, rect)
    rx = rect[0] - 2.5
    ry = rect[1] + 2.5
    rh, rw = 5, 5
    px = point[0]
    py = point[1]

    if (px >= rx and px <= rx + rw and py >= ry and py <= ry + rh):
        return True
    return False


def gen_traj(pos0, vel0, acc0, posf, velf, accf):
    
    '''
    Based on initial and final position, velocity and acceleration to generate a time stamped trajectory
    '''
    
    global traj
    
    traj._axis = [SingleAxisTrajectory(pos0[i],vel0[i],acc0[i]) for i in range(3)]
    traj.set_goal_position(posf)
    traj.set_goal_velocity(velf)
    traj.set_goal_acceleration(accf)
    
    # Run the algorithm, and generate the trajectory.
    traj.generate(Tf)
    print("Trajectory Generated")

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
    
    '''
    
    Callback to read pad velocity and set the final/goal position, velocity and acceleration
    
    '''
    
    global pos0, vel0, acc0, posf, velf, accf, t_init, counter, landing_mode, landing_executed
    latest_poses    = msg.latest_poses
    latest_velocity = msg.latest_velocities 
    avg_x, avg_y, avg_z, avg_vx, avg_vy, avg_vz =0,0,0,0,0,0
    
    if(landing_mode == True):
        delta_t=2
    else: 
        delta_t = 1
    
    for i in range(len(latest_poses)):
        avg_x = avg_x + latest_poses[i].pose.position.x 
        avg_y = avg_y + latest_poses[i].pose.position.y
        avg_z = avg_z + latest_poses[i].pose.position.z
        avg_vx = avg_vx + latest_velocity[i].vx
        avg_vy = avg_vy + latest_velocity[i].vy
        avg_vz = avg_vz + latest_velocity[i].vz

    avg_vx = avg_vx/len(latest_poses)
    avg_vy = avg_vy/len(latest_poses)
    avg_vz = avg_vz/len(latest_poses)
    
    x = avg_x/len(latest_poses) + avg_vx*delta_t
    y = avg_y/len(latest_poses) + avg_vy*delta_t
    
    z = landing_threshold
    
    posf = [x, y, z]
    velf = [avg_vx, avg_vy, avg_vz]
        
    counter = counter + 1 
    if( counter>100 and landing_executed == False ):
        counter= 0
        t_init = rospy.get_time()
        gen_traj(pos0, vel0, acc0, posf, velf, accf)
        print('posf and velf: ', posf, velf)
    
    
def callback_quad(msg):
    '''
    
    Callback from setting inital position and velocity from ground truth quadrotor states
    
    '''
    global pos0,vel0,posf, landing_threshold, landing_mode, landing_executed
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    z=msg.pose.pose.position.z    
    pos0 = [x,y,z]
    vx =msg.twist.twist.linear.x
    vy =msg.twist.twist.linear.y
    vz =msg.twist.twist.linear.z
    vel0 = [vx, vy, vz]
    
    if(abs(z-landing_threshold)<0.1 and landing_mode == False):
        print("Landing mode == On")
        landing_threshold = 0
        landing_mode = True

    if(landing_mode == True and abs(z) < 0.5):
        landing_executed = True
        

sub_quadstate =  rospy.Subscriber('/ground_truth/state', Odometry, callback_quad, queue_size = 10)
sub_padstate  = rospy.Subscriber('/pad_velocity', PosesAndVelocities, callback_pad, queue_size = 10)
pub_pos = rospy.Publisher('/command/pose',  PoseStamped, queue_size = 10)
pub_vel = rospy.Publisher('/command/twist', TwistStamped, queue_size = 10)
pub_vel_msg = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

pos_goal_msg = PoseStamped()
vel_goal_msg = Twist()

msg = rospy.wait_for_message("/pad_velocity", PosesAndVelocities , timeout=5)

t_init = rospy.get_time()    
gen_traj(pos0, vel0, acc0, posf, velf, accf)
flag_publisher = rospy.Publisher('/pid_tuner',  Bool, queue_size = 10)


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

flag=Bool()
flag.data = True

for i in range(10):
    flag_publisher.publish(flag)

delta_t=0.1

while not rospy.is_shutdown():
    flag_publisher.publish(flag)
    #Generate Trajectories
    t = rospy.get_time()

    if(t-t_init < Tf):
        try:
    
            velocity_ = traj.get_velocity(t - t_init)
            vel_goal_msg.linear.x = velocity_[0]
            vel_goal_msg.linear.y = velocity_[1]
            vel_goal_msg.linear.z = velocity_[2]
            
            position_ = traj.get_position(t- t_init + delta_t)
            pos_goal_msg.header.frame_id = 'world'
            pos_goal_msg.pose.position.x = position_[0]
            pos_goal_msg.pose.position.y = position_[1]
            pos_goal_msg.pose.position.z = position_[2]
            
            #Plotting Code
            time[i]        = t - t_init
            position[i, :] = traj.get_position(t - t_init)
            velocity[i, :] = traj.get_velocity(t - t_init)
            acceleration[i, :] = traj.get_acceleration(t - t_init)
            thrust[i] = traj.get_thrust(t - t_init)
            ratesMagn[i] = np.linalg.norm(traj.get_body_rates(t - t_init))
            i = i + 1
            
            timestampsList.append(t)
            distanceFromGround.append(pos0[2]) 
            xline.append(pos0[0])
            yline.append(pos0[1])

            pub_pos.publish(pos_goal_msg)
            
            #Shut down the quadrotor if landing was executed
            if(landing_executed == True):
                print("Shutting Down Quadrotor")
                vel_goal_msg.linear.x = 0
                vel_goal_msg.linear.y = 0
                vel_goal_msg.linear.z = 0
                pub_vel_msg.publish(vel_goal_msg)
                ret = subprocess.call(['rosservice call /enable_motors "enable: false"'], shell=True)
                break
                
        except:
            print("Error")
            continue
        
    else:
       if(landing_executed == True):
           break
       i=0

timeVsDist(timestampsList, distanceFromGround)
trajectory3D(xline, yline, distanceFromGround)
ret = pidTuner(time, position, velocity, acceleration, thrust, ratesMagn, Tf, fmin, fmax, wmax)
flag.data = ret
flag_publisher.publish(flag)

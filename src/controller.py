from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj

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
 
def init():
	#Function for describing all the publishers and subscribers
	#Get current pose and time
	rospy.Subscriber('/downward_cam/camera/image', Image, callback)



if __name__ == '__main__':
	init()

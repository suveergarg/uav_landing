# uav_landing

### Change Log ###

Removed <gazebo plugins> from hector_quadrotor_description/urdf/quadrotor_downward_cam.gazebo.xacro
Change size of Quadrotor in :


Components:

1. Setup Platform Movement and Simulation environment in Gazebo.

2. AR tag tracking for position and movement information about the platform.

3. Visual odometry with IMU? For state estimation of robot. (SVO, PTAMP. Should we implement from scratch?)

4. Planner to generate plans based on this information. Straight line? (Available packages in ROS 2D?)

5. Controller to execute these plans.(PID/ LQR?)


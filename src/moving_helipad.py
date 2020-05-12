#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates


x_vel = 1.0
y_vel = 0.5
model_name = ""
model_state_pub = None
model_state_sub = None


waypoints = [[10,10],[10,-10],[20,-10],[20,10],[10,10],[10,-10],[0,-10],[0,10]]

waypoints = [[-10,10],[10,10],[10,-10],[-10,-10]]
#xvel = [2, 0, -2, 0]
#yvel = [0, -2, 0, 2]

xvel = [0, 1, 0, -1]
yvel = [1, 0, -1, 0]

waypoints_index = 0

def init():
    
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub
    rospy.init_node('moving_helipad')
    model_name = rospy.get_param("~model_name", "marker3")
    x_vel = rospy.get_param("~x_vel", 1.0)
    y_vel = rospy.get_param("~y_vel", 1.0)
    seconds_before_moving = rospy.get_param("~seconds_before_moving", 10)
    rospy.sleep(seconds_before_moving)
    model_state_sub = rospy.Subscriber("gazebo/model_states", ModelStates, modelStatesCallback)
    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()

def modelStatesCallback(msg):
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub, waypoints_index,waypoints
    index_of_interest = -1

    for i in range(len(msg.name)):
        if msg.name[i] == model_name:
            index_of_interest = i
            break

    if index_of_interest >= 0:
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = msg.pose[index_of_interest]
        twist = Twist()
        
    '''
	if(model_state.pose.position.y>5 or model_state.pose.position.y<-5):
		y_vel = -1* y_vel

=======
        
    if index_of_interest >= 0:
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = msg.pose[index_of_interest]
        twist = Twist()
       
    
        if(model_state.pose.position.y>15 or model_state.pose.position.y<-15):
               y_vel = -1* y_vel
      
>>>>>>> e083c2e34ed73dce429937896c4dddc13e6e2284
        twist.linear.x = x_vel
        twist.linear.y = y_vel
        if msg.twist[index_of_interest] != twist:
            model_state.twist = twist
            model_state_pub.publish(model_state)
<<<<<<< HEAD
    '''

    # twist = Twist()
    
    
    
    # if(abs(model_state.pose.position.x - waypoints[waypoints_index][0]) < 0.1 and abs(model_state.pose.position.y - waypoints[waypoints_index][1])<0.1):
    #     waypoints_index = (waypoints_index + 1) % 4
    #     # print(waypoints_index, waypoints[waypoints_index][0] - waypoints[waypoints_index-1][0], waypoints[waypoints_index][1] - waypoints[waypoints_index-1][1])
    #     if(waypoints[waypoints_index][0] - waypoints[waypoints_index-1][0] == 0):
    #         twist.linear.x = 0.0 
    #     elif(waypoints[waypoints_index][0] > 0):
    #         twist.linear.x = abs(x_vel)
    #     else:
    #         twist.linear.x = -abs(x_vel)
        
    #     if(waypoints[waypoints_index][1] - waypoints[waypoints_index-1][1] == 0):
    #         twist.linear.y = 0.0 
    #     elif(waypoints[waypoints_index][1] > 0):
    #         twist.linear.y = abs(y_vel)
    #     else:
    #         twist.linear.y = -abs(y_vel)
            
    #     # print(waypoints_index, " ", twist.linear.x, " " ,twist.linear.y)     
    
      
    #     if msg.twist[index_of_interest] != twist:
    #         # print("here")
    #         model_state.twist = twist
    #         model_state_pub.publish(model_state)
    # else:
    #     # print("previous waypoint, index = %d"%waypoints_index)
    #     twist.linear.x = xvel[waypoints_index]
    #     twist.linear.y = yvel[waypoints_index]
    #     # print("velocity being set is ", xvel[waypoints_index], yvel[waypoints_index])
    #     model_state.twist = twist
    #     model_state_pub.publish(model_state)

    
if __name__ == '__main__':
    init()

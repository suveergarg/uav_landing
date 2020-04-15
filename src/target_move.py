# -*- coding: utf-8 -*-

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates


def init():
    rospy.init_node('moving_helipad')
    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    seconds_before_moving = rospy.get_param("~seconds_before_moving", 10)
    rospy.sleep(seconds_before_moving)

    while not rospy.is_shutdown():
        rospy.spin()
        
def modelStatesCallback(msg):
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub
    model_state = ModelState()
    model_state.model_name = model_name
    model_state.pose = msg.pose[index_of_interest]
    twist = Twist()
    twist.linear.x = x_vel
    twist.linear.y = y_vel
    if msg.twist[index_of_interest] != twist:
            model_state.twist = twist
            model_state_pub.publish(model_state)

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from ar_track_alvar.msgs import AlvarMarkers
from visualiation_msgs.msgs import Marker
pub = None

def callback(msg):
	pub.publish(msg)

def init():
  rospy.init_node('alvar_trackop')
  pub = rospy.Publisher()
  rospy.Subscriber('visualiation_marker', Marker, callback)
  
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  init()
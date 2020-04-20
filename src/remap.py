import rospy
from sensor_msgs.msg import Image

pub = rospy.Publisher('/downward_cam/camera/image_raw', Image, queue_size=10)
def callback(msg):
	pub.publish(msg)

def init():
  rospy.init_node('remap_node')
  rospy.Subscriber('/downward_cam/camera/image', Image, callback)
  
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  init()


import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

pub_twist = rospy.Publisher('/twistupdate', TwistWithCovarianceStamped, queue_size=10)
pub_pose = rospy.Publisher('/poseupdate',   PoseWithCovarianceStamped, queue_size=10)

def callback(msg):
	twist_msg = TwistWithCovarianceStamped()
	pose_msg  = PoseWithCovarianceStamped()	

	pose_msg.pose  = msg.pose
	twist_msg.twist = msg.twist 
	
	pub_pose.publish(pose)
	pub_twist.publish(twist)

def init():
  rospy.init_node('sensor_fusion_node')
  rospy.Subscriber('/mono_odometer/odometry', Image, callback)
  
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  init()


#!/usr/bin/env python
"""
https://answers.ros.org/question/321501/subscriber-python-camerainfo-and-image/
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
https://answers.ros.org/question/145801/cant-locate-node-in-package/
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

def callback(data):
  print('got here')
  # try:
  #   cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
  # except CvBridgeError as e:
  #   print(e)

  # (rows,cols,channels) = cv_image.shape
  # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
  # parameters =  aruco.DetectorParameters_create()
  # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
  # frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
  # print("plotting")
  # plt.figure()
  # # plt.imshow(frame_markers)
  # for i in range(len(ids)):
  #     c = corners[i][0]
  #     plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
  # plt.legend()
  # plt.savefig("aurco_marker_detection.png")


def init():
  rospy.init_node('camera_feed_proecssor')
  rospy.Subscriber('/downward_cam/camera/image', Image, callback)
  while not rospy.is_shutdown():
      rospy.spin()

if __name__ == '__main__':
  init()

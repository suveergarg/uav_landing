#!/usr/bin/env python

"""
https://answers.ros.org/question/321501/subscriber-python-camerainfo-and-image/
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
https://answers.ros.org/question/145801/cant-locate-node-in-package/
https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics_video.html
"""
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

def callback(data):
  # print('got here')
  bridge = CvBridge()
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)

  (rows,cols,channels) = cv_image.shape
  cv2.imwrite("/home/pooja/Desktop/catkin_ws/src/uav_landing/src/color.png", cv_image)
  # print('rows and columns ', rows, cols, channels, cv_image)
  gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

  (rows,cols) = gray.shape
  # plt.imshow(gray)
  cv2.imwrite("/home/pooja/Desktop/catkin_ws/src/uav_landing/src/gray.png", gray)
  # print('rows and columns ', gray)
  aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
  print('aruco dict ', aruco_dict)
  parameters =  aruco.DetectorParameters_create()
  print('parameters', parameters)
  corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
  frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)
  
  print("corners: {}, ids: {}, rejectedImgPoints: {} ".format(corners, ids, rejectedImgPoints))
  # print("plotting")
  # plt.figure()
  # # plt.imshow(frame_markers)
  # for i in range(len(ids)):
  #     c = corners[i][0]
  #     plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
  # plt.legend()
  

  plt.figure()
  plt.imshow(frame_markers, origin = "upper")
  if ids is not None:
    for i in range(len(ids)):
      c = corners[i][0]
      
      plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "+", label = "id={0}".format(ids[i]))
    plt.savefig("/home/pooja/Desktop/catkin_ws/src/uav_landing/src/aurco_marker_detection.png")


def init():
  rospy.init_node('camera_feed_processor')
  rospy.Subscriber('/downward_cam/camera/image', Image, callback)
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  init()

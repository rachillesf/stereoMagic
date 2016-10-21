#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import time
import scipy

#change it to your own global path to workspace
GLOBAL_PATH = '/home/rachillesf/catkin_ws'


class CameraSim:

  def __init__(self):
    self.left_pub = rospy.Publisher("/camera/left/image_raw",Image,queue_size=1)
    self.right_pub = rospy.Publisher("/camera/right/image_raw",Image,queue_size=1)
    self.left = cv2.imread(GLOBAL_PATH +  '/src/stereo/sample_images/left.png',0)
    self.right = cv2.imread(GLOBAL_PATH + '/src/stereo/sample_images/right.png',0)
    self.bridge = CvBridge()

  def pub(self):
    try:
      self.left_pub.publish(self.bridge.cv2_to_imgmsg(self.left))
    except CvBridgeError as e:
      print(e)

    try:
      self.right_pub.publish(self.bridge.cv2_to_imgmsg(self.right))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('camera_simulator', anonymous=True)
  cs = CameraSim()

  while(1):
    cs.pub()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

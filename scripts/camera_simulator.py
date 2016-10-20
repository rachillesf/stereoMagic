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


class image_converter:

  def __init__(self):
    self.left_pub = rospy.Publisher("/camera/left/image_raw",Image,queue_size=1)
    self.right_pub = rospy.Publisher("/camera/right/image_raw",Image,queue_size=1)
    self.left = cv2.imread('src/stereo/sample_images/left.png',0)
    self.right = cv2.imread('src/stereo/sample_images/right.png',0)
    self.bridge = CvBridge()

    print("Simulator Disparity Node Initialized")

  def pub(self):

    try:
      self.left_pub.publish(self.bridge.cv2_to_imgmsg(self.left))
      print("publishing1")
    except CvBridgeError as e:
      print(e)

    try:
      self.right_pub.publish(self.bridge.cv2_to_imgmsg(self.right))
      print("publishing2")
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('camera_simulator', anonymous=True)
  ic1 = image_converter()

  while(1):
    ic1.pub()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

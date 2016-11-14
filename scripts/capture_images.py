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
#cv2.namedWindow("window", cv2.WINDOW_AUTOSIZE) 
class Rectify:

  def __init__(self):
  
    self.bridge = CvBridge()
    self.rimage_sub = message_filters.Subscriber("/camera/right/image_raw",Image)
    self.limage_sub = message_filters.Subscriber("/camera/left/image_raw",Image)
    self.ts = message_filters.TimeSynchronizer([self.limage_sub, self.rimage_sub], 1).registerCallback(self.callback)
    self.counter = 0
   


  def callback(self,left,right):

    try:
      cv_image_left = CvBridge().imgmsg_to_cv2(left)
    except CvBridgeError as e:
      print(e)

    try:
      cv_image_right = CvBridge().imgmsg_to_cv2(right)
    except CvBridgeError as e:
      print(e)
	
    if self.counter % 100 == 0:
    	cv2.imwrite("images/left" + str(self.counter/100) + ".jpg", cv_image_left)
    	cv2.imwrite("images/right" + str(self.counter/100) + ".jpg", cv_image_right)	
	print "salvando..."
	print self.counter
	#cv2.imshow("window",cv_image_left)
	#cv2.waitKey(3)

    self.counter += 1


def main(args):
  rospy.init_node('rectify', anonymous=True)
  ic = Rectify()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

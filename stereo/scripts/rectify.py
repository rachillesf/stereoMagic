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
    self.left_pub = rospy.Publisher("/camera/left/ret",Image,queue_size=1)
    self.right_pub = rospy.Publisher("/camera/right/ret",Image,queue_size=1)
    self.bridge = CvBridge()
    self.rimage_sub = message_filters.Subscriber("/camera/right/image_raw",Image)
    self.limage_sub = message_filters.Subscriber("/camera/left/image_raw",Image)
    self.ts = message_filters.TimeSynchronizer([self.limage_sub, self.rimage_sub], 1).registerCallback(self.callback)
    self.dist = np.load('src/stereo/params/dist.npy')
    self.mtx = np.array(np.load('src/stereo/params/mtx.npy'))
    self.ret = [np.load('src/stereo/params/ret.npy')]
    self.rvecs = np.load('src/stereo/params/rvecs.npy')
    self.tvecs = np.load('src/stereo/params/tvecs.npy')
    print("Rectify Node Initialized")



  def retify(self,img):
    #img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img = cv2.resize(img, (900,600), interpolation = cv2.INTER_AREA)
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),0,(w,h))
    # undistort
    mapx,mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,newcameramtx,(w,h),5)
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    return dst


  def callback(self,left,right):

    try:
      cv_image_left = CvBridge().imgmsg_to_cv2(left)
    except CvBridgeError as e:
      print(e)

    try:
      cv_image_right = CvBridge().imgmsg_to_cv2(right)
    except CvBridgeError as e:
      print(e)

    left = cv2.pyrDown(self.retify(cv_image_left))
    right = cv2.pyrDown(self.retify(cv_image_right))


    try:
      self.left_pub.publish(self.bridge.cv2_to_imgmsg(left, "8UC1"))
    except CvBridgeError as e:\
      print(e)

    try:
      self.right_pub.publish(self.bridge.cv2_to_imgmsg(right, "8UC1"))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_converter1', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

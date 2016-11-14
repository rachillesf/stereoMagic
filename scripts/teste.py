#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image",Image,self.callback)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    try:
      image_camera = rospy.wait_for_message('/camera/left/image_raw', Image, timeout=2)
      image = self.bridge.imgmsg_to_cv2(image_camera, "bgr8")
      print type(image)

    except CvBridgeError as e:
      print(e)
     detectPeople(cv_image_left)
    cv2.imshow("Image window", image)
    cv2.waitKey(3)






def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

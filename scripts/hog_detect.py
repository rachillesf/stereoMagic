#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
from hoglib import *

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image",Image,self.callback)
    self.marker = Marker()
    self.publisher = rospy.Publisher("/pessoa", Marker)

  def callback(self,depth_msg):
    try:
      depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "bgr8")
    except CvBridgeError as e:
      print(e)

    try:
      camera_msg = rospy.wait_for_message('/camera/left/image_raw', Image, timeout=0.3)
      image_camera = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")

    except CvBridgeError as e:
      print(e)


    boxes, image = detectPeople(image_camera)
    depth_image = cv2.resize(depth_image, (640,480), interpolation=cv2.INTER_AREA)
    if(boxes is not None):
        for (xA, yA, xB, yB) in boxes:
            if boxes.dtype.kind == "i":
                boxes = boxes.astype("float")

            x_center= int(((xA-xB)/2) + xB )
            y_center= int(((yA-yB)/2) + yB )
            #cv2.circle(image,(x_center,y_center), 40, (0,0,255), -1)

            #print np.shape(depth_image)
            #print np.shape(image)
            #print "caixa: ", (xA, yA, xB, yB)
            #print "centro: ", (x_center,y_center)
            #print "intensidade ", depth_image[y_center ,x_center]


            self.marker.header.frame_id = "stereo_frame"
            self.marker.ns = "basic_shapes";
            self.marker.id = 0;


            self.marker.type = self.marker.CUBE
            self.marker.action = self.marker.ADD
            self.marker.scale.x = float(xB - xA)/10.0
            self.marker.scale.y = float(yB - yA)/10.0
            self.marker.scale.z = 5

            self.marker.color.a = 0.3
            self.marker.color.r = 0.0
            self.marker.color.g = 0.0
            self.marker.color.b = 1.0
            self.marker.pose.orientation.w = 1.0

            self.marker.pose.position.x = x_center
            self.marker.pose.position.y = y_center
            self.marker.pose.position.z = depth_image[y_center ,x_center][0]
            self.publisher.publish(self.marker)




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

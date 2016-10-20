#!/usr/bin/env python
import roslib
import rospy
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

#from __future__ import print_function

class find_wrench:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.ct = 0
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cascade = cv2.CascadeClassifier('/home/jonathan/wrench.xml')
    cv_image = cv2.resize(cv_image, (0,0), fx= 4, fy=4);
    rects = cascade.detectMultiScale(cv_image, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (35, 200))
    #print 'I am trying!'
    if len(rects) == 0:
        cents = np.array([0],np.float32)
        #print 'Did not find any!'
    else:
        rects[:, 2:] += rects[:, :2]
        cents = np.transpose(np.array([(rects[:,0]+rects[:,2])/2,(rects[:,1]+rects[:,3])/2],dtype=np.float32))
        #print rects
        cents = np.reshape(cents,(np.size(cents),1))
        print cents
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (127, 255, 0), 2)
    wrenchpub = rospy.Publisher('/wrench_centroids', numpy_msg(Floats), queue_size=5)
    wrenchpub.publish(cents)
    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    cv_image = cv2.resize(cv_image, (0,0), fx=0.25, fy=0.25);
    #print self.ct
    #if self.ct % 10 == 0:
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)
    self.ct = self.ct+1
    

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('find_wrench', anonymous=True)
  ic = find_wrench()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

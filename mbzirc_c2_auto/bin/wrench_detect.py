#!/usr/bin/env python
import roslib
import rospy
import rospkg
import sys
import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

#from __future__ import print_function

class find_wrench:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)
    self.w = rospy.Publisher("/wrench_center",numpy_msg(Floats), queue_size=1)
    self.ct = 0
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/mybot/camera1/image_raw",Image,self.callback)
    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    rospack = rospkg.RosPack()

    cascade = cv2.CascadeClassifier(rospack.get_path('mbzirc_c2_auto')+'/params/wrench.xml')
    cv_image = cv2.resize(cv_image, (0,0), fx=2, fy=2);
    rects = cascade.detectMultiScale(cv_image, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (35, 200))
    cv_image2 = cv_image.copy()
    #print 'I am trying!'
    if len(rects) == 0:
        cents = np.array([0],np.float32)
        x_avg = 0
        y_avg = 0
        #print 'Did not find any!'
    else:
        rects[:, 2:] += rects[:, :2]
        cents = np.transpose(np.array([(rects[:,0]+rects[:,2])/2,(rects[:,1]+rects[:,3])/2],dtype=np.float32))
        print rects
        cents = np.reshape(cents,(np.size(cents),1))
        #print cents
        x_avg = 0
        y_avg = 0
        ct = 0
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(cv_image2, (x1, y1), (x2, y2), (127, 255, 0), 2)
            x_avg = x_avg + (x1+x2)/2
            y_avg = y_avg + (y1+y2)/2
            ct = ct+1
        x_avg = x_avg/ct
        y_avg = y_avg/ct
    wrenchpub = rospy.Publisher('/wrench_centroids', numpy_msg(Floats), queue_size=5)
    wrenchpub.publish(cents)

    cimg = cv2.medianBlur(cv_image,5)
    cimg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(cimg, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=20, maxRadius=200)

    if circles is not None:
        mn = min(circles[0,:,0])
        idx = np.argwhere(circles[0,:,0] == mn)
        i = circles[0,:][idx][0][0]

        val_loc = np.array([i[0],i[1],i[2]], dtype=np.float32)
        w_loc = np.array([x_avg,y_avg], dtype=np.float32)
    # Publish /valve topic
        val = rospy.Publisher("/valve",numpy_msg(Floats), queue_size=1)
        val.publish(val_loc)
        cv2.circle(cv_image2,(i[0],i[1]),i[2],(0,0,255),2)

    else:
        w_loc = np.array([0],np.float32)

    self.ct = self.ct+1
    cv_image2 = cv2.resize(cv_image2, (0,0), fx=0.5, fy=0.5);

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
      self.w.publish(w_loc)

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

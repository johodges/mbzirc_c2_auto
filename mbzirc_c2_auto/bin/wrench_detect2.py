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

    scale_factor = 2
    ret,thresh = cv2.threshold(cv_image,105,255,0)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
    cv_image2 = cv_image.copy()
    if len(contours) == 0:
       cents = np.array([0],np.float32)
       x_avg = 0
       y_avg = 0
    else:
#        rects[:, 2:] += rects[:, :2]
        #print cents
        x_avg = 0
        y_avg = 0
        ct = 0
        i = 0
	for cnt in contours:

          x,y,w,h = cv2.boundingRect(cnt)
          if h > 70 and h < 200:
            cv2.rectangle(cv_image2,(x,y),(x+w,y+h),(0,255,0),2)
#			cv2.rectangle(frame,(x,y),(x+w,y+h/5),(0,0,255),2)
#array for centroids of wrenches needs to be corrected
            cents[i,:] = np.transpose(np.array([(x+x+w)/2,(y+y+h)/2],dtype=np.float32))
            i = i + 1
            x_avg = x_avg + (2*x+w)/2
            y_avg = y_avg + (2*y+h)/2
            ct = ct+1
  	    crop = cv_image[y-20:y+h/5,x:x+w]
     	    crop = cv2.resize(crop,(0,0),fx=scale_factor,fy=scale_factor)
	    crop1 = cv2.medianBlur(crop,5)
	    circles = cv2.HoughCircles(crop1, cv2.cv.CV_HOUGH_GRADIENT,1,10,np.array([]),100,30,0,0)
	    if circles != None:
	      a,b,c = circles.shape
	      for i in range(b):
	        cv2.circle(crop,(circles[0][i][0], circles[0][i][1]), circles[0][i][2], (255,0,0), 1, cv2.CV_AA)
                cv2.circle(crop,(circles[0][i][0], circles[0][i][1]), 2, (255,100,0), 1, cv2.CV_AA)
	        x0 = int(circles[0][i][0]/scale_factor)
	        y0 = int(circles[0][i][1]/scale_factor)
# Size of the circles
	    #print circles[0][i][2]/scale_factor
	        cv2.circle(cv_image2,(x0+x, y0+y-20), int(circles[0][i][2]/scale_factor), (255,0,0), 2, cv2.CV_AA)
	        cv2.circle(cv_image2,(x0+x, y0+y-20), 2, (255,100,0), 1, cv2.CV_AA)		       

          x_avg = x_avg/ct
          y_avg = y_avg/ct
        cents = np.reshape(cents,(np.size(cents),1))

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
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(crop, "bgr8"))
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

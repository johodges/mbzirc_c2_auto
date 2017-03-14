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
    self.image_pub = rospy.Publisher("/output/wrench_cascade_image",Image, queue_size=1)
    self.image_output = rospy.Publisher("/output/keyevent_image",Image, queue_size=1)
    self.w = rospy.Publisher("/wrench_center",numpy_msg(Floats), queue_size=1)
    self.ct = 0
    self.ct2 = 0
    self.bridge = CvBridge()
    self.indir = '/home/jonathan/'
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback, queue_size=1,buff_size=2**24)
    #self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback, queue_size=1,buff_size=2**24)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    rospack = rospkg.RosPack()
    cascade = cv2.CascadeClassifier(rospack.get_path('mbzirc_c2_auto')+'/params/wrench.xml')
    cv_image = cv2.resize(cv_image, (0,0), fx=2, fy=2);
    img_invert = 255 - cv_image
    rects = cascade.detectMultiScale(img_invert, 3, 3, cv2.cv.CV_HAAR_SCALE_IMAGE, (25, 200))
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
        # print rects
        cents = np.reshape(cents,(np.size(cents),1))
        cents = cents/2
        #print cents
        x_avg = 0
        y_avg = 0
        ct = 0
        sz = np.shape(rects)
        locs = np.empty([sz[0],2])
        width = np.empty([sz[0],2])
        locs[:,0] = (rects[:,2]+rects[:,0])/2
        locs[:,1] = (rects[:,3]+rects[:,1])/2
        width[:,0] = rects[:,2]-rects[:,0]
        width[:,1] = rects[:,3]-rects[:,1]
        locs2 = np.empty([len(locs[:,0]),2])
        width2 = np.empty([len(width[:,0]),2])
        #print "LOCS:"
        #print np.power(np.power(locs[:,1]-locs[1,1],2),0.5)
        for i in range(0,sz[0]):
            dist = np.power(np.power(locs[:,0]-locs[i,0],2),0.5)
            dist[dist == 0] = 99999
            """
            dist_min = np.min(np.extract(dist>0,dist))
            dist_arg = np.argwhere(dist==dist_min)
            dist_min = np.min(np.extract(dist>0,dist))
            """
            dist_arg = np.argwhere(dist<10)
            if dist_arg.size == 0:
                locs2[ct,:] = locs[i,:]
                width2[ct,:] = width[i,:]
                ct = ct+1
            else:
                if np.min(dist_arg) > i:
                    loc_temp = np.mean(locs[dist_arg,0])
                    locs2[ct,:] = [np.mean(locs[dist_arg,0]),np.mean(locs[dist_arg,1])]
                    width2[ct,:] = [np.mean(width[dist_arg,0]),np.mean(width[dist_arg,1])]
                    ct = ct+1
        width = np.abs(np.floor(width2[0:ct,:]))
        locs = np.floor(locs2[0:ct,:])
        rects2 = np.transpose(np.array([locs[:,0]-width[:,0]/2,locs[:,1]-width[:,1]/2,locs[:,0]+width[:,0]/2,locs[:,1]+width[:,1]/2]))
        rects = np.int_(rects2)
        rects2 = []
        for x1, y1, x2, y2 in rects:
            area = (x2-x1)/2*(y2-y1)/2
            #print
            if area > 1000:
                cv2.rectangle(cv_image2, (x1, y1), (x2, y2), (127, 255, 0), 8)
                x_avg = x_avg + (x1+x2)/2
                y_avg = y_avg + (y1+y2)/2
                ct = ct+1
                rects2.append(x1/2)
                rects2.append(y1/2)
                rects2.append(x2/2)
                rects2.append(y2/2)
        x_avg = x_avg/ct
        y_avg = y_avg/ct
    #print rects2
    wrenchpub = rospy.Publisher('/wrench_centroids', numpy_msg(Floats), queue_size=5)
    wrenchpub.publish(cents)
    #wrenchboxpub = rospy.Publisher('/wrench_bounding_box', numpy_msg(Floats), queue_size=5)
    #wrenchboxpub.publish(np.array(rects2,dtype=np.float32))
    w_loc = np.array([x_avg,y_avg], dtype=np.float32)

    cimg = cv2.medianBlur(cv_image,5)
    cimg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)

    #print np.shape(cents)[0]
    if np.shape(cents)[0] > 4*2:
        circles = cv2.HoughCircles(cimg, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=20, maxRadius=200)
        if circles is not None:
            mn = min(circles[0,:,0])
            idx = np.argwhere(circles[0,:,0] == mn)
            i = circles[0,:][idx][0][0]
            val_loc = np.array([i[0],i[1],i[2]], dtype=np.float32)
            # Publish /valve topic
            val = rospy.Publisher("/valve",numpy_msg(Floats), queue_size=1)
            val.publish(val_loc)
            cv2.circle(cv_image2,(i[0],i[1]),i[2],(0,0,255),2)
        #else:
        #    w_loc = np.array([0],np.float32)
    self.ct = self.ct+1
    cv_image2 = cv2.resize(cv_image2, (0,0), fx=0.5, fy=0.5);

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
      self.w.publish(w_loc)
      #cv2.imwrite('%scascade_result2.png' % (self.indir),cv_image2)
      if np.shape(cents)[0] > 6 and self.ct2 == 0:
        self.image_output.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
        self.ct2 = 1

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

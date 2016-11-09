#!/usr/bin/env python

import roslib; roslib.load_manifest('urg_node')
import rospy
import sensor_msgs.msg
import matplotlib.pyplot as plt
import numpy as np
import scipy as sc
import scipy.signal as sg
import scipy.misc as ms
import scipy.spatial.distance as scd
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

def callback(data):
    rate = rospy.Rate(10)
    #x = np.arange(4.71239,0,-0.004363323)
    #x = np.arange(2.35619,-2.358,-0.004363323)
    #x = np.arange(-2.35619,2.358,0.0065540750511)
    x = np.arange(1.66,-1.6525,-0.0029146999)
    thresh = 0.1
    xsin = np.sin(x)
    xcos = np.cos(x)
    plt.cla()
    y = sg.medfilt(data.ranges,1)
    y_diff1 = np.power(np.diff(y),2)
    x_coord = y*xsin
    y_coord = y*xcos
    x_diff = np.power(np.diff(x_coord),2)
    y_diff = np.power(np.diff(y_coord),2)
    dist = np.power(x_diff+y_diff,0.5)
    x2 = np.array(np.split(x, np.argwhere(dist > thresh).flatten()[1:]))
    y2 = np.array(np.split(y, np.argwhere(dist > thresh).flatten()[1:]))
    dist2 = np.array(np.split(dist, np.argwhere(dist > thresh).flatten()[1:]))
    x_coord2 = np.array(np.split(x_coord, np.argwhere(dist > thresh).flatten()[1:]))
    y_coord2 = np.array(np.split(y_coord, np.argwhere(dist > thresh).flatten()[1:]))
    for i in range(len(x2)):
        xlen = len(x2[i])-0
        if xlen > 4:

            #x_diff[x_diff > 0.1] = 0
            #y_diff[y_diff > 0.1] = 0
            dist2_sum = np.sum(dist2[i][1:xlen-1])
            if dist2_sum < 0.25:
                #plt.plot(x2[i][1:xlen],y2[i][1:xlen],'k-')
                #plt.plot(x2[i],y_coord2[i],'ks')
                hihi = 1
            else:
                if dist2_sum > 5:
                    #plt.plot(x2[i][1:xlen],y2[i][1:xlen],'b-')
                    #plt.plot(x2[i],y_coord2[i],'bs')
                    hihi = 1
                else:
                    #plt.plot(x2[i][1:xlen],y2[i][1:xlen],'r-')
                    #plt.plot(x2[i],y_coord2[i],'rs')
                    ang = np.median(x2[i])
                    dis = np.median(y2[i])
                    mn = min(y2[i][1:xlen])
                    mx = max(y2[i][1:xlen])
                    bearing = np.array([ang,dis], dtype=np.float32)

    if 'bearing' in locals():
        hihi = 1
    else:
        bearing = np.array([0,0], dtype=np.float32)
    pub = rospy.Publisher("/detection",numpy_msg(Floats), queue_size=1)
    pub.publish(bearing)
    #print bearing
    #plt.ylim([0,40])
    #plt.xlim([-20,20])
    #plt.ylabel('Distance [m]')
    #plt.xlabel('Distance [m]')
    #plt.title('Laser Scan')
    #plt.draw()
    #print 'Finished once...'
    #raw_input("Press Enter to continue...")
    pass
def laser_listener():
    pass
    rospy.init_node('findbox', anonymous=True)
    rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,callback, queue_size=1)
    #rospy.Subscriber("/hokuyo/laser/most_intense",sensor_msgs.msg.LaserScan,callback, queue_size=1)
    rospy.spin()
    #print

if __name__ == '__main__':

    #plt.ion()
    rospy.loginfo('Looking for oject...')
    print "Looking for object..."
    laser_listener()

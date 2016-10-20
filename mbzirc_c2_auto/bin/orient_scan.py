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
    y = np.arange(1.66,-1.6525,-0.0029146999)
    thresh = 0.1
    ysin = np.sin(y)
    ycos = np.cos(y)
    plt.cla()
    x = sg.medfilt(data.ranges,1)
    x_diff1 = np.power(np.diff(x),2)
    y_coord = x*ysin
    x_coord = x*ycos
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
                    #plt.plot(x_coord2[i][1:xlen],y_coord2[i][1:xlen],'r-')
                    ang = np.median(y2[i])
                    #print ang
                    #print x2
                    idx = np.argwhere(y2[i] == ang)
                    if len(idx) == 0:
                        #print np.size(x2[i])
                        ang = np.median(y2[i][1:xlen])
                        idx = np.argwhere(y2[i] == ang)
                    #print idx
                    xA = x_coord2[i][idx]
                    yA = y_coord2[i][idx]
                    dis = x2[i][idx]
                    mn = min(x2[i][1:xlen])
                    idx2 = np.argwhere(x2[i] == mn)
                    mx = max(x2[i][1:xlen])
                    rng = mx-mn
                    xB = x_coord2[i][idx2]
                    yB = y_coord2[i][idx2]
                    angB = y2[i][idx2]
                    m = -1*(xB-xA)/(yB-yA)
                    xmn = min(x_coord2[i][1:xlen])
                    xmx = max(x_coord2[i][1:xlen])
                    ymn = min(y_coord2[i][1:xlen])
                    ymx = max(y_coord2[i][1:xlen])
                    #print xA, yA, xB, yB
                    #print abs(xA-xB), abs(yA-yB)
                    if rng > 0.0001:
                        if xA > xB:
                            bearing = np.array([-0.08,xA,yA,xB,yB,xmn,xmx,ymn,ymx], dtype=np.float32)
                        else:
                            bearing = np.array([0.08,xA,yA,xB,yB,xmn,xmx,ymn,ymx], dtype=np.float32)
                        if yA > yB:
                            bearing[0] = bearing[0]*-1
                    #bearing = np.array([ang], dtype=np.float32)
                    
    if 'bearing' in locals():
        hihi = 1
    else:
        bearing = np.array([0,0], dtype=np.float32)
    pub = rospy.Publisher("/bearing",numpy_msg(Floats), queue_size=1)
    pub.publish(bearing)
    #print bearing
    #plt.ylim([0,2])
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
    rospy.init_node('orient_scan', anonymous=True)
    rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,callback, queue_size=1)
    #rospy.Subscriber("/hokuyo/laser/most_intense",sensor_msgs.msg.LaserScan,callback, queue_size=1)
    rospy.spin()
    #print 

if __name__ == '__main__':
    
    #plt.ion()
    print "Looking for object..."
    laser_listener()

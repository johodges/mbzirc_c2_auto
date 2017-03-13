#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import urllib2
import urllib2 as ul
import time 
import cv2


ip_right = 'http://10.10.10.13/?command=ptz_req&req=start&param=directionleft&channel=1&stream=0'
ip_left = 'http://10.10.10.13/?command=ptz_req&req=start&param=directionright&channel=1&stream=0'
ip_up = 'http://10.10.10.13/?command=ptz_req&req=start&param=directiondown&channel=1&stream=0'
ip_down = 'http://10.10.10.13/?command=ptz_req&req=start&param=directionup&channel=1&stream=0'
ip_zoomIn = 'http://10.10.10.13/?command=ptz_req&req=start&param=zoomtile&channel=1&stream=0'
ip_zoomOut = 'http://10.10.10.13/?command=ptz_req&req=start&param=zoomwide&channel=1&stream=0'
ip_stop = 'http://10.10.10.13/?command=ptz_req&req=stop&param=directionright&channel=1&stream=0'


# All the url commands are commented since the code was developed without the actaul camera
# be sure to uncomment them in actual use.
def control_PTZ(pan,tilt,zoom_in,zoom_out):
	# Define all the command
	if(pan>0 and tilt==0):
		print 'turn left'
		#response = ul.urlopen(ip_left)
	elif(pan<0 and tilt==0):
		print 'turn right'
		#response = ul.urlopen(ip_right)
	elif(pan==0 and tilt>0):
		print 'turn up'
		#response = ul.urlopen(ip_up)
	elif(pan==0 and tilt<0):
		print 'turn down'
		#response = ul.urlopen(ip_down )
	elif(pan==0 and tilt==0 and zoom_in==0 and zoom_out==0):
		#response = ul.urlopen(ip_stop )
                x=1
        elif(pan==0 and tilt==0):
		if(zoom_in==1 and zoom_out==0):
                	print 'zoom in'
			#response = ul.urlopen(ip_zoomIn)
		elif(zoom_in==0 and zoom_out==1):
			print 'zoom out'
			#response = ul.urlopen(ip_zoomOut)
def callback(data):
	#rospy.loginfo(data)
	#rospy.loginfo('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
	pan = data.axes[3] 		      #pan 	
	tilt = data.axes[4]                       #tilt
        zoom_in = data.buttons[3]
        zoom_out = data.buttons[1]
	control_PTZ(pan,tilt,zoom_in,zoom_out)   
def listener():
    rospy.init_node('teleop', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()
if __name__ == '__main__':
    listener()

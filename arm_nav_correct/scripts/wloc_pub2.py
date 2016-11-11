#!/usr/bin/env python
# license removed for brevity
# temperate node to publish location of wrench to test correction step
   
import rospy
import time
import actionlib
import math
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class LocationPub():
    def __init__(self, node_name):
  	self.node_name = node_name
	rospy.init_node(node_name)
	rospy.loginfo("Starting node " + str(node_name))
	#rospy.on_shutdown(self.cleanup)

    def image_callback(self, data):
	self.wloc_pub()

    def wloc_pub(self):
	wlocs = [0.5, 0.5, 0.2]
    	self.wlocs = PoseStamped()
    	self.wlocs.point.x = wlocs[0]
    	self.wlocs.point.y = wlocs[1]
    	self.wlocs.point.z = wlocs[2]

    	self.poi_pub.publish(self.wlocs)
    	
def main(args):
    try:
        node_name = "LocationPub"
        LocationPub(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down LocationPub node."
        cv.DestroyAllWindows()  
if __name__ == '__main__': 
    main(sys.argv) 

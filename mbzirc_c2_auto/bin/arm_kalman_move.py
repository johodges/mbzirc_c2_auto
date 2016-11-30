#!/usr/bin/env python
import roslib
import rospy
import rospkg
import sys
from geometry_msgs.msg import Twist
from mbzirc_c2_auto.msg import kf_msg
import numpy as np
import scipy.linalg as linalg


class arm_kalman:

  def __init__(self):
    self.est_pose = rospy.Publisher("/move_arm/valve_pos_kf",Twist, queue_size=1)
    self.input_pose = rospy.Subscriber("/move_arm/valve_pos",Twist,self.gen_kf_position,queue_size=1)
    self.kf_init = rospy.Subscriber("/move_arm/kf_init",kf_msg,self.init_KF,queue_size=1)

    zeroPos = np.array([0.0 , 0.0])
    initCv = 0.01 * np.eye(2)

    self.mx_kmGkm = zeroPos
    self.Sx_kmGkm = np.eye(2)
    self.Sw_km = initCv
    self.Sv_k = initCv
    self.u = np.array([0.0 , 0.0])

    self.ready_to_init = True


  def gen_kf_position(self,data):
    '''Callback to generate a new filtered position
    '''
    rospy.loginfo("Computing the RBE for valve move.")
    I = np.eye(2)

    # Obtain the current measured valve position
    zk = np.array([data.linear.y , data.linear.z])


    # Prediction Step
    mx_kGkm = self.mx_kmGkm + self.u
    Sx_kGkm = self.Sx_kmGkm + self.Sw_km

    # Correction Step
    K_den = Sx_kGkm + self.Sv_k
    Kk = linalg.solve(K_den.T , Sx_kGkm.T)
    Kk = Kk.T
    mx_kGk = mx_kGkm + (Kk.dot(zk - mx_kGkm))
    tmp = (I - Kk)
    Sx_kGk = tmp.dot(Sx_kGkm)

    rospy.logdebug("u       = %6.3f | %6.3f", self.u[0], self.u[1])
    rospy.logdebug("z_k     = %6.3f | %6.3f", zk[0], zk[1])
    rospy.logdebug("mx_kmkm = %6.3f | %6.3f", self.mx_kmGkm[0], self.mx_kmGkm[1])
    rospy.logdebug("Sx_kmkm = %6.3f | %6.3f", self.Sx_kmGkm[0,0], self.Sx_kmGkm[1,1])
    rospy.logdebug("mx_kkm  = %6.3f | %6.3f", mx_kGkm[0], mx_kGkm[1])
    rospy.logdebug("Sx_kkm  = %6.3f | %6.3f", Sx_kGkm[0,0], Sx_kGkm[1,1])
    rospy.logdebug("K       = %6.3f | %6.3f", Kk[0,0], Kk[1,1])
    rospy.logdebug("mx_kk   = %6.3f | %6.3f", mx_kGk[0], mx_kGk[1])
    rospy.logdebug("Sx_kk   = %6.3f | %6.3f", Sx_kGk[0,0], Sx_kGk[1,1])

    curr_pose = data
    rospy.loginfo("Original (x,y,z) pose:      (%6.4f, %6.4f, %6.4f)",
                  curr_pose.linear.x,
                  curr_pose.linear.y,
                  curr_pose.linear.z)

    # Publish estimated valve position
    est_valve_pose = data
    est_valve_pose.linear.y = mx_kGk[0]
    est_valve_pose.linear.z = mx_kGk[1]

    rospy.loginfo("New estimated (x,y,z) pose: (%6.4f, %6.4f, %6.4f)",
                  est_valve_pose.linear.x,
                  est_valve_pose.linear.y,
                  est_valve_pose.linear.z)

    self.est_pose.publish(est_valve_pose)
    rospy.sleep(0.1)

    # Populate values for the next step
    self.u = - mx_kGk
    self.mx_kmGkm = mx_kGk
    self.Sx_kmGkm = Sx_kGk




  def init_KF(self,init_data):
    '''Callback to initialize the Kalman filter
    '''
    rospy.loginfo("Initializing the Kalman filter")
    self.mx_kmGkm = np.array([init_data.initial_pos[1],init_data.initial_pos[2]])
    self.Sx_kmGkm = np.diag(init_data.initial_covar)
    self.Sw_km = np.diag(init_data.covar_motion)
    self.Sv_k = np.diag(init_data.covar_observer)
    self.u = np.array([0.0 , 0.0])
    rospy.logdebug(init_data)


def main(args):
  rospy.init_node('arm_kalman', anonymous=True, log_level=rospy.DEBUG)
  # rospy.init_node('arm_kalman', anonymous=True, log_level=rospy.INFO)
  kf = arm_kalman()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

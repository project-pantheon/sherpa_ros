#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist

class Ackermann(object):
  def __init__(self):
    self.sub = rospy.Subscriber('/ackrm_cmd_vel', Twist, self.sub_callback)
    self.data = Twist()
    
  def sub_callback(self, msg):
    self.data = msg

    
class publisher(object):
  def __init__(self, pub, sub_msg):
    self._pub = pub
    self.data = sub_msg

  def callback(self,sub_msg):
    self.data = sub_msg

    
def main():

  rospy.init_node('cmd_vel_ackrm_node', anonymous=True)
  rate = rospy.Rate(10)
  akrm = Ackermann()
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  
  h = 1.335
  while not rospy.is_shutdown():

    vx = akrm.data.linear.x
    phi = akrm.data.angular.z

    if phi != 0.0:
        R = h/math.tan(abs(phi))
        omega = (vx/R)*np.sign(phi)
    else:
        R = 0.0
        omega = 0.0


    cmd_msg = publisher(pub, akrm.data)
    cmd_msg.data.linear.x = vx
    cmd_msg.data.angular.z = omega
    cmd_msg._pub.publish(cmd_msg.data)

    rate.sleep()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
  
  
  

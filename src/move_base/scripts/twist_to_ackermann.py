#!/usr/bin/env python
#coding=utf-8

import rospy
import sys
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from math import sqrt, pow, atan2, copysign

class Converter:
  def __init__(self, topic):
  
    self.ackermann_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
    self.cmd_vel_topic = topic
    
    #pub ackermann msg
    self.pub = rospy.Publisher(self.ackermann_topic, AckermannDriveStamped,queue_size=1)
    #sub to cmd_vel topic
    rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_callback)
    
    rospy.spin()
  	
  def _cmd_vel_callback(self, msg):
    print "\nvx:", msg.linear.x,"vy:", msg.linear.y,"vth:", msg.angular.z	

    #speed = sqrt(vx^2 + vy^2)
    #R = speed / vth and tan(angle) = L / R 
    #angle = atan(L/R) = atan(L * vth / speed) = atan(L * msg.angular.z / speed)
    speed = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2)) 
    #前后轮轴间距（m）urdf中有定义
    L = 0.2
    
    #[00 -, 00],confirm the range of angular
    steering_angle = atan2(msg.angular.z * L, speed)
    if steering_angle > 1.3:
    	steering_angle = 1.3
    if steering_angle < -1.3:
    	steering_angle = -1.3
  	
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.drive.speed = speed
    msg.drive.acceleration = 1
    msg.drive.jerk = 0
    msg.drive.steering_angle = steering_angle
    msg.drive.steering_angle_velocity = 0
    self.pub.publish(msg)
  	

if __name__=="__main__":
  try:

    parameters = sys.argv[1:]
    if len(parameters) is not 1:
      print('Try: rosrun move_base twist_to_ackermann.py <topic name>')
      sys.exit(-1)
    
    topic = None
    if parameters[0] in ('/computed_control_actions', '/raw_cmd_vel'):
      topic = parameters[0]
    else:
     sys.exit(-1)
    
    rospy.init_node("twist_to_ackermann")
    Converter(topic)
  except rospy.ROSInterruptException:
    pass
    
    
    
    
    
    
    
    
    
    
    
    
    
    

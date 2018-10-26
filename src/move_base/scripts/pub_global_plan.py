#!/usr/bin/env python

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy

if __name__=="__main__":
  
  rospy.init_node('path')
  pub = rospy.Publisher('/move_base/GlobalPlanner/plan', Path, queue_size=1)
  
  msg = Path()
  msg.poses = [PoseStamped() for i in range(100)]
  for i in range(100):
    msg.poses[i].pose.position.x = i
    msg.poses[i].pose.position.y = i*2
  
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()
  
  rospy.spin()
    
  
  
  
  






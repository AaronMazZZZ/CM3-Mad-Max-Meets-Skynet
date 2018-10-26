#!/usr/bin/env python
#coding=utf-8
import rospy
from nav_msgs.msg import Path
from math import sqrt, pow, atan2, copysign

#convert the global plan to pid path
class PlanTracker:
  def __init__(self):
  
    self.first_planner_flag = True;
    self.planner_topic = "/move_base/GlobalPlanner/plan"
    
    #pub ackermann msg
    #self.pub = rospy.Publisher(self.ackermann_topic, AckermannDriveStamped,queue_size=1)
    
    #sub global planner
    rospy.Subscriber(self.planner_topic, Path, self._plan_callback)
    
    rospy.spin()
  	
  def _plan_callback(self, msg):
    if self.first_planner_flag:
       self.first_planner_flag = False;
    else: 
       return
    
    print "received", len(msg.poses), "poses"
  	

if __name__=="__main__":
  try:
    rospy.init_node("PlanTracker")
    PlanTracker()
  except rospy.ROSInterruptException:
    pass
    
    
    
    
    
    
    
    
    
    
    
    
    
    

#!/usr/bin/env python
#coding=utf-8
from .trajectory import Trajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion
import tf
import rospy
from math import cos, sin

class RacecarTrajectory(object, Trajectory):
    def __init__(self, path):
        Trajectory.__init__(self)
        self.poses = path.poses
        self.current_position = Point()
        self.current_pose_id = 0
        self.id = 0
        self.listener = tf.TransformListener()
        
        self.sample_path()
        
        for i in range(len(self.poses)):
          print("x:",self.poses[i].pose.position.x,"y:",self.poses[i].pose.position.y)
        print("create racecar_trajectory")
       

    def get_position_at(self, i, current_position):
        self.current_position = current_position
        try:
          (trans, rot) = self.listener.lookupTransform('/odom', '/map', rospy.Time(0))
          #trans pose from ref /map to ref /odom            
          if self.id < len(self.poses):
            tmp_x = self.poses[self.id].pose.position.x
            tmp_y = self.poses[self.id].pose.position.y
            euler = euler_from_quaternion(rot)
            angle = euler[2]
            self.position.x = tmp_x * cos(angle) - tmp_y * sin(angle) + trans[0]
            self.position.y = tmp_y * cos(angle) + tmp_x * sin(angle) + trans[1]
        
          print("len:", len(self.poses))
          print("next goal id:", self.id, "x:", self.position.x, "y:", self.position.y)
          print("robot pose:", "x:", current_position.x, "y:", current_position.y)
        
          self.id += 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("error")

        return self.position

    def get_name(self):
        return str(RacecarTrajectory.__name__).replace('Trajectory', '').lower()
    
    #因为全局规划的点间距很大，需要进行插值
    def sample_path(self):
        tmp = PoseStamped()
        pose_len = len(self.poses)
        self.tmp_poses = []
        for i in range(pose_len-1):
          tmp.pose.position.x = (self.poses[i].pose.position.x + self.poses[i].pose.position.x)*0.5
          tmp.pose.position.y = (self.poses[i+1].pose.position.y + self.poses[i+1].pose.position.y)*0.5
          self.tmp_poses.append(self.poses[i])
          self.tmp_poses.append(tmp)
          i += 2
        self.tmp_poses.append(self.poses[-1])
        self.poses = self.tmp_poses
        
    def get_closest_point(self):
        min_dis = 10000
        min_pose_id = 0
        tmp_point = Point()
        for i in range(len(self.poses)):
          tmp_point.x = self.poses[i].pose.position.x
          tmp_point.y = self.poses[i].pose.position.y
          dis = self.get_dis(tmp_point, self.current_position)
          if min_dis > dis:
            min_dis = dis
            min_pose_id = i
        return min_pose_id
        
    def get_dis(self, point0, point1):
        return (point0.x - point1.x) * (point0.y - point1.y)
        
    def delete_passed_poses(self):
        last_point = Point()
        last_point.x = self.poses[-1].pose.position.x
        last_point.y = self.poses[-1].pose.position.y
        dis_robot_to_the_last_point = self.get_dis(last_point, self.current_position)
        tmp_point = Point()
        for p in self.poses:
          tmp_point.x = p.pose.position.x
          tmp_point.y = p.pose.position.y
          dis_pose_to_the_last_point = self.get_dis(tmp_point, last_point)
          if dis_pose_to_the_last_point > dis_robot_to_the_last_point:
            self.poses.remove(p)
          
          
          
          
          
          
          
          
          
          
          
          
          
          

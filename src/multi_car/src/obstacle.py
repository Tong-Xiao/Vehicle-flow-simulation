#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
import sys, getopt, math


class ObstacleControl():
  def __init__(self):
    self.LASER_ERR = 0.05
    self.IN_CROSS_FLAG=0
    self.ns=rospy.get_param('nssss')
    self.move_cmd = Twist()
    rospy.init_node('obstacle_control_node', anonymous=True)
    self.ambulance_control_pub = rospy.Publisher('/{vehicle_name}/cmd_vel'.format(vehicle_name=self.ns),  Twist, queue_size=10)
    self.angular_sub = rospy.Subscriber('/{vehicle_name}/camera_front/error'.format(vehicle_name=self.ns), Float64, self.angular_callback)
    self.in_cross_flag_sub = rospy.Subscriber("/{vehicle_name}/camera_front/in_cross_flag".format(vehicle_name=self.ns), Float64, self.in_cross_flag_callback)
    self.laser_sub = rospy.Subscriber('/{vehicle_name}/front_laser/scan'.format(vehicle_name=self.ns), LaserScan, self.laser_callback)
    
  
  def angular_callback(self,msg):
    self.angular=msg
    self.move_cmd.angular.z = -self.angular.data
    

  def in_cross_flag_callback(self,msg):
    self.IN_CROSS_FLAG=msg
    

  def laser_callback(self,msg):
    self.scan_filter = []
    self.scan_filter_1 = []
    for i in range(180):
      if i <= 95 and i>= 85:
        if msg.ranges[i] >= self.LASER_ERR:
          self.scan_filter.append(msg.ranges[i])
    for i in range(180):
      if i <= 170 and i>= 10:
        if msg.ranges[i] >= self.LASER_ERR:
          self.scan_filter_1.append(msg.ranges[i])
    if min(self.scan_filter) < 20:
                self.move_cmd.linear.x = self.move_cmd.linear.x-0.05
                if min(self.scan_filter) < 5:
                   self.move_cmd.linear.x=0
    else:
                self.move_cmd.linear.x = self.move_cmd.linear.x+0.05

                if self.move_cmd.linear.x > 4.5:
                  self.move_cmd.linear.x=4.5

    if self.IN_CROSS_FLAG==1:
      if min(self.scan_filter_1) < 8:
                self.move_cmd.linear.x =self.move_cmd.linear.x-0.1


                    
                # rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))

  def obstacle(self):
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown(): 
      if abs(self.move_cmd.angular.z) > 1.1 and self.move_cmd.linear.x>1:
        self.move_cmd.linear.x=1
      elif abs(self.move_cmd.angular.z) > 0.8 and self.move_cmd.linear.x>2:
        self.move_cmd.linear.x=2
      elif abs(self.move_cmd.angular.z) > 0.05 and self.move_cmd.linear.x>2.5:
        self.move_cmd.linear.x=2.5
      
      if self.IN_CROSS_FLAG==1 and self.move_cmd.linear.x>1:
        self.move_cmd.linear.x=1
      ##############################
      # self.move_cmd.linear.x=0
      if self.move_cmd.linear.x < 0:
          self.move_cmd.linear.x=0
      if self.move_cmd.angular.z == 0:
          self.move_cmd.angular.z=0.005
      self.ambulance_control_pub.publish(self.move_cmd)
      rate.sleep()
 
if __name__ == '__main__':
    Obsracle=ObstacleControl()
    try:
      rate = rospy.Rate(50)
      
      Obsracle.obstacle()
      rate.sleep()

    except rospy.ROSInterruptException:
      pass
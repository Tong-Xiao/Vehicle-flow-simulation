#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
import sys, getopt, math


def MultiCarControl():
    rospy.init_node('multi_car_control_node', anonymous=True)
    ambulance_control_pub=rospy.Publisher('/ambulance/cmd_vel',  Twist, queue_size=10)
    bus_control_pub=rospy.Publisher('/bus/cmd_vel',  Twist, queue_size=10)
    suv_control_pub=rospy.Publisher('/suv/cmd_vel',  Twist, queue_size=10)
    pickup_control_pub=rospy.Publisher('/pickup/cmd_vel',  Twist, queue_size=10)
    fire_truck_control_pub=rospy.Publisher('/fire_truck/cmd_vel',  Twist, queue_size=10)
    prius_control_pub=rospy.Publisher('/prius/cmd_vel',  Twist, queue_size=10)
    #add car example step3
    ambulance_1_control_pub=rospy.Publisher('/ambulance_1/cmd_vel',  Twist, queue_size=10)

    rate = rospy.Rate(50)
    move_cmd = Twist()
    prius_move_cmd = Twist()

    while not rospy.is_shutdown():
     move_cmd.linear.x=5.0
     move_cmd.angular.z=0.0
     prius_move_cmd.linear.x=7.0
     prius_move_cmd.angular.z=0.0
     ambulance_control_pub.publish(move_cmd)
     bus_control_pub.publish(move_cmd)
     suv_control_pub.publish(move_cmd)
     pickup_control_pub.publish(move_cmd)
     fire_truck_control_pub.publish(move_cmd)
     prius_control_pub.publish(prius_move_cmd)
     #add car example step 4
     ambulance_1_control_pub.publish(move_cmd)

     rate.sleep()  

if __name__ == '__main__':
    try:
      MultiCarControl()
    except rospy.ROSInterruptException:
      pass
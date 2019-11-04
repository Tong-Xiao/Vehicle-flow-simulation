#!/usr/bin/env python
# coding=utf-8

import sys
from PIL import Image
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
import rospy
from cv_bridge import CvBridge, CvBridgeError
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
import random
import math

import matplotlib.image as mplimg
import matplotlib.pyplot as plt

blur_ksize = 5  # Gaussian blur kernel size
canny_lthreshold = 50  # Canny edge detection low threshold
canny_hthreshold = 150  # Canny edge detection high threshold


# Hough transform parameters
rho = 1
theta = np.pi / 180
threshold = 15
min_line_len = 40
max_line_gap = 20
color=[255, 0, 0]
thickness=8

ONLYONCEFLAG=1
class Follower:
    def __init__(self):
        rospy.init_node('camera_calc_node', anonymous=True)
        self.CROSSBRANCH=0
        self.TargetBranch=0
        self.FIND_TARGET_BRANCH_FLAG=0
        self.ONLYONCEFLAG=1
        self.IN_CROSS_FLAG=0
        self.front_right_indix=0
        self.front_left_indix=0
        #self.image = mplimg.imread("/home/ubuntu/DeskTop/camera_test/src/camera_test/data/cross_3.jpg")
        self.ns=rospy.get_param('nssss')
        if self.ns=='pickup' or self.ns=='bus':
            self.AddLength=2
        else:
            self.AddLength=0
        self.bridge =CvBridge()
        self.image_sub = rospy.Subscriber("/{vehicle_name}/camera_front/image_raw".format(vehicle_name=self.ns), Image, self.image_callback)
        self.position_sub = rospy.Subscriber("/{vehicle_name}/odom".format(vehicle_name=self.ns), Odometry, self.position_callback)
        self.angular_pub = rospy.Publisher("/{vehicle_name}/camera_front/error".format(vehicle_name=self.ns), Float64, queue_size=2)
        self.in_cross_flag_pub = rospy.Publisher("/{vehicle_name}/camera_front/in_cross_flag".format(vehicle_name=self.ns), Float64, queue_size=2)
        self.Sum_P=0
        self.Sum_I=0
        self.Sum_D=0
        self.Last_Sum_P=0
        self.lastcenterline=320
        self.cross_odom=[(0.5,4.5),(0,402),(-400,-147),(0,-351)]
        # print(self.cross_odom[0][1])
        self.cross_branch=[[1,2,3,4],[1,3,4],[2,3,4],[1,2]]
        self.position_x=0
        self.position_y=0
        self.orientation_x=0
        self.orientation_y=0
        self.orientation_z=0
        self.orientation_w=0
        self.angle=0
        self.headposition_x=0
        self.headposition_y=0
        self.TargetPosition_x=0.0
        self.TargetPosition_y=0.0
        if self.ns=='prius':
            self.LocationKP = 0.2
            self.LocationKI = 0.0
            self.LocationKD = 0
            self.bias=150
        elif self.ns=='pickup':
            self.LocationKP = 0.25
            self.LocationKI = 0.0
            self.LocationKD = 0
            self.bias=150
        elif self.ns=='suv' or self.ns=='suv_1':
            self.LocationKP = 0.26
            self.LocationKI = 0.0
            self.LocationKD = 0.0
            self.bias=150
        else:
            self.LocationKP = 0.28
            self.LocationKI = 0.0
            self.LocationKD = 0.15
            self.bias=90
    
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_an_image()
    
    def position_callback(self, msg):
        self.position_x=msg.pose.pose.position.x
        self.position_y=msg.pose.pose.position.y
        self.orientation_x=msg.pose.pose.orientation.x
        self.orientation_y=msg.pose.pose.orientation.y
        self.orientation_z=msg.pose.pose.orientation.z
        self.orientation_w=msg.pose.pose.orientation.w
        self.angle=math.atan2(2*self.orientation_w*self.orientation_z+2*self.orientation_x*self.orientation_y,1-2*(self.orientation_y*self.orientation_y+self.orientation_z*self.orientation_z))

        if self.ns=='pickup' or self.ns=='fire_truck' or self.ns=='suv' or self.ns=='suv_1' :
            self.angle=self.angle+3.14


        if self.angle>3.14:
            self.angle=self.angle-6.28
        elif self.angle<-3.14:
            self.angle=self.angle+6.28
        # print(self.angle)

    
    def denoise_road(self):
        #从两边向中间去除白色噪声区域
        for i in range(0,self.ImageCC.shape[0]-1,1):
            LeftWhiteCount=0
            RightWhiteCount=0
            for j in range(0,self.ImageCC.shape[1]-1):
                if self.ImageCC[i,j]>250:
                    self.ImageCC[i,j]=0
                else:
                    LeftWhiteCount=LeftWhiteCount+1
                if LeftWhiteCount>12:
                    break
            for j in range(self.ImageCC.shape[1]-1,0,-1):
                if self.ImageCC[i,j]>250:
                    self.ImageCC[i,j]=0
                else:
                    RightWhiteCount=RightWhiteCount+1
                if RightWhiteCount>12:
                    break

        #去除左上角区域，避免识别左侧车道线
        if self.lastcenterline>320:
            for i in range(0,90,1):
                for j in range(0,int(-3*i+self.lastcenterline-self.bias+20)):
                    self.ImageCC[i,j]=100
        else:
            for i in range(0,90,1):
                for j in range(0,int(-2*i+self.lastcenterline-self.bias)):
                    self.ImageCC[i,j]=100
    
    def find_central_road(self):
        LeftWhiteCount=0
        RightWhiteCount=0
        centralline=0.0
        rightflag=0
        leftflag=0
        rightexistanceflag=0
        leftexistanceflag=0
        count=0
   
        #从中间向两边找车道线
        for i in range(20,self.ImageCC.shape[0]-35,2):
            for j in range(self.lastcenterline,self.ImageCC.shape[1]-1,2):
                if self.ImageCC[i,j]>250:
                    RightWhiteCount=RightWhiteCount+1
                    if RightWhiteCount>8:
                        rightflag=1
                        rightexistanceflag=1
                        rightline=j
                        RightWhiteCount=0
                        break
                    else:
                        continue
                
            for j in range(self.lastcenterline,0,-1):
                if self.ImageCC[i,j]>250:
                    LeftWhiteCount=LeftWhiteCount+1
                if LeftWhiteCount>8:
                    leftflag=1
                    leftexistanceflag=1
                    leftline=j
                    LeftWhiteCount=0

                    break
                else:
                    continue
            if leftflag and rightflag:
                count=count+1
                centralline=centralline+(rightline+leftline)/2
                rightflag=0
                leftflag=0

        leftlosscount=0
        rightlosscount=0
        if count != 0:
            centralline=centralline/count
        elif rightexistanceflag:
            leftlosscount=leftlosscount+1
            if leftlosscount>10:
                centralline=350
                leftlosscount=0
            else:
                centralline=320
        elif leftexistanceflag:
            rightlosscount=rightlosscount+1
            if rightlosscount>10:
                centralline=280
                rightlosscount=0
            else:
                centralline=320
        else:
            centralline=320

        self.lastcenterline=int(centralline)
        # print(centralline)

        self.center=(centralline-320)/320
    
    def if_cross(self):
        
        x_bias=[self.cross_odom[x][0]-self.position_x for x in range(0,4)]
        y_bias=[self.cross_odom[y][1]-self.position_y for y in range(0,4)]
        #路口CROSSFLAG
        for i in range(0,4):
            if abs(x_bias[i])+abs(y_bias[i])<15+self.AddLength:
                CROSSFLAG=i
                self.IN_CROSS_FLAG=1
                break
            else:
                CROSSFLAG=4
                self.IN_CROSS_FLAG=0
            # print(CROSSFLAG)
        #在十字路口范围内确定所在分支self.CROSSBRANCH
        if CROSSFLAG in range(0,4) :
            if self.FIND_TARGET_BRANCH_FLAG==0 or self.CROSSBRANCH==0:
                self.FIND_TARGET_BRANCH_FLAG=1
                if x_bias[CROSSFLAG]>=0 and x_bias[CROSSFLAG]<=13+self.AddLength and y_bias[CROSSFLAG]>=0 and y_bias[CROSSFLAG]<=4:
                    self.CROSSBRANCH=1
                elif x_bias[CROSSFLAG]>=0 and x_bias[CROSSFLAG]<=4 and y_bias[CROSSFLAG]>=-13-self.AddLength and y_bias[CROSSFLAG]<=0:
                    self.CROSSBRANCH=2
                elif x_bias[CROSSFLAG]>=-13-self.AddLength and x_bias[CROSSFLAG]<=0 and y_bias[CROSSFLAG]>=-4 and y_bias[CROSSFLAG]<=0:
                    self.CROSSBRANCH=3
                elif x_bias[CROSSFLAG]>=-4 and x_bias[CROSSFLAG]<=0 and y_bias[CROSSFLAG]>=0 and y_bias[CROSSFLAG]<=13+self.AddLength:
                    self.CROSSBRANCH=4
                else:
                    self.CROSSBRANCH=0

            #目标分支以及坐标（每个路口只计算一次）
                if self.CROSSBRANCH>0:
                    #目标分支
                    self.TargetBranch=random.sample(self.cross_branch[CROSSFLAG],1)
                    while self.TargetBranch[0]==self.CROSSBRANCH:
                        self.TargetBranch=random.sample(self.cross_branch[CROSSFLAG],1)

                    
                    #确定目标位置坐标
                    if self.TargetBranch[0]==1:
                        self.TargetPosition_x=self.cross_odom[CROSSFLAG][0]-20
                        self.TargetPosition_y=self.cross_odom[CROSSFLAG][1]+2
                        self.TargetOrientation=-1.57  
                        
                        print(1)  
                    elif self.TargetBranch[0]==2:
                        self.TargetPosition_x=self.cross_odom[CROSSFLAG][0]+2
                        self.TargetPosition_y=self.cross_odom[CROSSFLAG][1]+20
                        self.TargetOrientation=3.14
                        
                        print(2)
                    elif self.TargetBranch[0]==3:
                        self.TargetPosition_x=self.cross_odom[CROSSFLAG][0]+20
                        self.TargetPosition_y=self.cross_odom[CROSSFLAG][1]-2
                        self.TargetOrientation=1.57
                        
                        print(3)
                    elif self.TargetBranch[0]==4:
                        self.TargetPosition_x=self.cross_odom[CROSSFLAG][0]-2
                        self.TargetPosition_y=self.cross_odom[CROSSFLAG][1]-20
                        self.TargetOrientation=0
                        
                        print(4)
                    
            #计算目标位置与车辆的夹角           
            if self.CROSSBRANCH>0:
                if self.CROSSBRANCH==1:
                    datedef=math.atan2(self.TargetPosition_y-self.position_y,self.TargetPosition_x-self.position_x)
                if self.CROSSBRANCH==2:
                    datedef=math.atan2(self.TargetPosition_x-self.position_x,-self.TargetPosition_y+self.position_y)
                if self.CROSSBRANCH==3:
                    datedef=math.atan2(-self.TargetPosition_y+self.position_y,-self.TargetPosition_x+self.position_x)
                if self.CROSSBRANCH==4:
                    datedef=math.atan2(-self.TargetPosition_x+self.position_x,self.TargetPosition_y-self.position_y)
                # print(self.CROSSBRANCH,self.cross_odom[CROSSFLAG],(self.TargetPosition_x,self.TargetPosition_y)) 
                
                #直走
                angle_bias=self.angle-self.TargetOrientation
                if angle_bias>3.14:
                    angle_bias=angle_bias-6.28
                elif angle_bias<-3.14:
                    angle_bias=angle_bias+6.28

                if abs(self.CROSSBRANCH-self.TargetBranch[0])==2:
                    self.center=(-datedef)*1+angle_bias*2
                #左拐
                elif self.TargetBranch[0]-self.CROSSBRANCH in [1,-3]:
                    if self.ns=='prius':
                        if datedef>1.14:
                            self.center=((angle_bias)-(-1.57+datedef))*3.8 
                        else:
                            self.center=0
                    elif self.ns=='pickup':
                        if datedef>1.12:
                            self.center=((angle_bias)-(-1.57+datedef))*3.8 
                        else:
                            self.center=0
                    elif self.ns=='suv' or self.ns=='suv_1':
                        if datedef>1.13:
                            self.center=((angle_bias)-(-1.57+datedef))*3.8 
                        else:
                            self.center=0
                    else:
                        if datedef>1.12:
                            self.center=((angle_bias)-(-1.57+datedef))*3.8  
                        else:
                            self.center=0 
                    

                #右拐
                elif self.TargetBranch[0]-self.CROSSBRANCH in [-1,3]:
                    
                    if self.ns=='pickup':
                        if datedef<-1.08:
                            self.center=((angle_bias)-(datedef+1.57))*15
                        else:
                            self.center=0
                    elif self.ns=='prius':
                        if datedef<-1.17:
                            self.center=((angle_bias)-(datedef+1.57))*10
                        else:
                            self.center=0
                    elif self.ns=='suv' or self.ns=='suv_1':
                        if datedef<-1.1:
                            self.center=((angle_bias)-(datedef+1.57))*16
                        else:
                            self.center=0      
                    else:
                        if datedef<-1.1:
                            self.center=((angle_bias)-(datedef+1.57))*13
                        else:
                            self.center=0
                  
                self.lastcenterline=320


                # print(self.CROSSBRANCH,self.TargetBranch[0],datedef,self.center) 

        else:
            self.FIND_TARGET_BRANCH_FLAG=0
            self.CROSSBRANCH=0
        if self.CROSSBRANCH==0:
            self.FIND_TARGET_BRANCH_FLAG=0

               


    def process_an_image(self):
        rate = rospy.Rate(100)

        gray = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0, 0)
        ret,binaryzation = cv2.threshold(blur_gray, 50, 255, cv2.THRESH_BINARY)
        cropped = binaryzation[128:240, 0:640]       


        kernel = np.ones((3,3),np.uint8)
        self.ImageCC=cv2.morphologyEx(cropped,cv2.MORPH_CLOSE,kernel)
        
        self.if_cross()

        if self.FIND_TARGET_BRANCH_FLAG==0:
            self.denoise_road()
            self.find_central_road()
            if self.ns=='prius':
                if self.center>0.03:
                    self.center=self.center*1.22
                elif self.center<-0.03 :
                    self.center=self.center*1.28
            else:
                if self.center>0.02:
                    self.center=self.center*1.225
                elif self.center<-0.02 :
                    self.center=self.center*1.28

            if self.center>1.0:
                self.center=1.0
            else:
                if self.center<-1.0:
                    self.center=-1.0

            
        angular_z=self.PIDcalc()
        self.angular_pub.publish(angular_z)
        self.in_cross_flag_pub.publish(self.IN_CROSS_FLAG)



        cv2.imshow("window", self.ImageCC)
        cv2.waitKey(3)
        rate.sleep()

    def PIDcalc(self):

        
        # print(center)
        error=self.center-0.0
        self.Sum_P = self.LocationKP * error	
        self.Sum_I =self.Sum_I+ self.LocationKI * error 
        self.Sum_D = ((self.Sum_P - self.Last_Sum_P)) * self.LocationKD 
        self.Last_Sum_P = self.Sum_P
        self.Proper_Location = self.Sum_P +self.Sum_I+self.Sum_D
        if self.Proper_Location>1:
            self.Proper_Location=1
        elif self.Proper_Location<-1:
            self.Proper_Location=-1
        
        self.Proper_Location=round(self.Proper_Location,6)
        # print(self.lastcenterline,self.center,self.Proper_Location)
        return self.Proper_Location


if __name__ == '__main__':

    
    try:
        follower = Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

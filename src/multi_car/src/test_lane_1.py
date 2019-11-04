#!/usr/bin/env python
# coding=utf-8

import sys
from PIL import Image
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
import rospy
from cv_bridge import CvBridge, CvBridgeError
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np

import matplotlib.image as mplimg
import matplotlib.pyplot as plt


blur_ksize = 5  # Gaussian blur kernel size
canny_lthreshold = 50  # Canny edge detection low threshold
canny_hthreshold = 150  # Canny edge detection high threshold


# Hough transform parameters
rho = 1
theta = np.pi / 180
threshold = 15
min_line_length = 40
max_line_gap = 20
color=[255, 0, 0]
thickness=8

class Follower:
    def __init__(self):
        rospy.init_node('camera_calc_node', anonymous=True)
        self.ns=rospy.get_param('nssss')
        self.bridge =CvBridge()
        self.image_sub = rospy.Subscriber("/{vehicle_name}/camera_front/image_raw".format(vehicle_name=self.ns), Image, self.image_callback)
        self.angular_pub = rospy.Publisher("/{vehicle_name}/camera_front/error".format(vehicle_name=self.ns), Float64, queue_size=2)
        self.Sum_P=0
        self.Sum_I=0
        self.Sum_D=0
        self.Last_Sum_P=0
        self.lastcenterline=320
        if self.ns=='prius' or self.ns=='pickup':
            self.LocationKP = 0.22
            self.LocationKI = 0.00001
            self.LocationKD = 0.15
            self.bias=190
        else:
            self.LocationKP = 0.28
            self.LocationKI = 0.00001
            self.LocationKD = 0.15
            self.bias=80

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_an_image()

    def process_an_image(self):
        rate = rospy.Rate(100)
        roi_vtx = np.array([[(0, self.image.shape[0]), (460, 325), (520, 325), (self.image.shape[1], self.image.shape[0])]])

        gray = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0, 0)
        ret,binaryzation = cv2.threshold(blur_gray, 50, 255, cv2.THRESH_BINARY)
        cropped = binaryzation[128:240, 0:640]


        kernel = np.ones((3,3),np.uint8)
        ImageCC=cv2.morphologyEx(cropped,cv2.MORPH_CLOSE,kernel)

        #从两边向中间去除白色噪声区域
        for i in range(0,ImageCC.shape[0]-1,1):
            LeftWhiteCount=0
            RightWhiteCount=0
            for j in range(0,ImageCC.shape[1]-1):
                if ImageCC[i,j]>250:
                    ImageCC[i,j]=0
                else:
                    LeftWhiteCount=LeftWhiteCount+1
                if LeftWhiteCount>12:
                    break
            for j in range(ImageCC.shape[1]-1,0,-1):
                if ImageCC[i,j]>250:
                    ImageCC[i,j]=0
                else:
                    RightWhiteCount=RightWhiteCount+1
                if RightWhiteCount>12:
                    break
        
        #去除左上角区域，避免识别左侧车道线
        if self.lastcenterline>310:
            for i in range(0,90,1):
                for j in range(0,int(-3*i+self.lastcenterline-self.bias+20)):
                    ImageCC[i,j]=100
        else:
            for i in range(0,90,1):
                for j in range(0,int(-2*i+self.lastcenterline-self.bias)):
                    ImageCC[i,j]=100


        
        LeftWhiteCount=0
        RightWhiteCount=0
        centralline=0.0
        rightflag=0
        leftflag=0
        rightexistanceflag=0
        leftexistanceflag=0
        count=0
 
          
        #从中间向两边找车道线
        for i in range(20,ImageCC.shape[0]-30,2):
            for j in range(self.lastcenterline,ImageCC.shape[1]-1,2):
                if ImageCC[i,j]>250:
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
                if ImageCC[i,j]>250:
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
                centralline=310
        elif leftexistanceflag:
            rightlosscount=rightlosscount+1
            if rightlosscount>10:
                centralline=280
                rightlosscount=0
            else:
                centralline=310
        else:
            centralline=310


        self.lastcenterline=int(centralline)

        center=(centralline-320)/320
        if center>0.01:
            center=center*1.2
        elif center<-0.01 :
            center=center*1.25
        if center>1.0:
            center=1.0
        else:
            if center<-1.0:
                center=-1.0
        # print(center)

        error=center-0.0
        

        angular_z=self.PIDcalc(error)
        
        self.angular_pub.publish(angular_z)

        cv2.imshow("window", self.image)
        cv2.waitKey(3)
        rate.sleep()

    def PIDcalc(self,error):
        self.Sum_P = self.LocationKP * error	
        self.Sum_I =self.Sum_I+ self.LocationKI * error 
        self.Sum_D = ((self.Sum_P - self.Last_Sum_P)) * self.LocationKD 
        self.Last_Sum_P = self.Sum_P
        self.Proper_Location = self.Sum_P +self.Sum_I+self.Sum_D
        if self.Proper_Location>1:
            self.Proper_Location=1
        elif self.Proper_Location<-1:
            self.Proper_Location=-1

        return self.Proper_Location


if __name__ == '__main__':

    
    try:
        follower = Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
     


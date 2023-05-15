#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from warper import Warper
from slide_window import SlideWindow


### 
# from warper import Warper
# from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import Float64, Int64, Bool
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd
# from sensor_msg.msg import CompressedImage
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path, Odometry
from math import *


class Controller() :

    def __init__(self) :
        rospy.init_node("line_detector")


        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.ImuCB) ## Vehicle Status Subscriber

        self.bridge = CvBridge()
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.original_img = []
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.waypoint_sub = rospy.Subscriber("/waypoint", Int64, self.waypointCB)
        self.mission_sub = rospy.Subscriber("/mission", Bool, self.mission_callback)
        self.yaw = 0.0
        self.waypoint = 0
        self.odom_msg = Odometry()

        # self.camera_steering_pub= rospy.Publisher('/cam_steer', CtrlCmd, queue_size=1)

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0.0
        self.current_lane_window = ""

        self.initialized = False # image_callback

        self.error_lane = 0
        self.steering_val = 0
        self.mission = False


        # gps 신호 받기 -> gps 끊길 때만 연산
        self.original_longitude = 0.0
        self.original_latitude = 0.0

        # For test
        self.motor_msg = 5.0
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            # self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
            self.ctrl_cmd_pub = rospy.Publisher('/cam_steer', CtrlCmd, queue_size=1)
            self.ctrl_cmd_msg = CtrlCmd()
            # self.motor_msg = 5.0
            self.servo_msg = 0.0
            self.brake_msg = 0.0
            self.ctrl_cmd_msg.longlCmdType = 2

            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            rate.sleep()




        # rospy.Subscriber("/image_jpeg/compressed", Image, self.image_callback)
        rospy.spin()
    
    # def runnung(self, _event) :

    def mission_callback(self, msg) :
        self.mission = msg.data
        # print('lane CB', self.mission)

    def waypointCB(self, msg) :
        self.waypoint = msg.data
        # print('way', self.waypoint)
        
        
    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude
    
    def ImuCB(self, msg) :
        self.odom_msg.pose.pose.orientation.x = msg.orientation.x
        self.odom_msg.pose.pose.orientation.y = msg.orientation.y
        self.odom_msg.pose.pose.orientation.z = msg.orientation.z
        self.odom_msg.pose.pose.orientation.w = msg.orientation.w

        quaternion = (self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w)
        _, _, self.yaw =  euler_from_quaternion(quaternion)
        self.yaw = degrees(self.yaw) 


    def image_callback(self, _data) :
        # print(type(_data))
        # if self.initialized == False:
        #     cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
        #     cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
        #     cv2.createTrackbar('low_S', 'Simulator_Image', 50, 255, nothing)
        #     cv2.createTrackbar('low_V', 'Simulator_Image', 50, 255, nothing)
        #     cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
        #     cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
        #     cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
        #     self.initialized = True

    

        if (self.original_longitude  <= 1 and self.original_latitude <=  1 and self.mission == True) or  (905 <= self.waypoint <=  990):
            print('lane')

            cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)
            # cv2.imshow("original", cv2_image)

            


            # low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
            # low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
            # low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
            # high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
            # high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
            # high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


            hsv_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
            # lower_lane = np.array([low_H, low_S, low_V]) 
            # upper_lane = np.array([high_H, high_S, high_V])

            lower_lane = np.array([0, 0, 126]) #0 46 170 / white lane: 0 0 126
            upper_lane = np.array([255, 64, 180]) #79 255 255 / white lane : 255, 64, 180


            lane_mask = cv2.inRange(hsv_image, lower_lane, upper_lane)

            ksize = 5
            blur_lane = cv2.GaussianBlur(lane_mask, (ksize, ksize), 0)

            _, lane_image = cv2.threshold(blur_lane, 200, 255, cv2.THRESH_BINARY)

            # cv2.imshow("Lane Image", lane_image)

            warper_image = self.warper.warp(lane_image)

            # cv2.circle(cv2_image,  (0, 450),5,(255,0,0),5) #w h
            # cv2.circle(cv2_image, (160, 300),5,(0,0,255),5)

            # cv2.circle(cv2_image, (480, 300),5,(255,255,0),5)
            # cv2.circle(cv2_image, (640, 450),5,(0,255,255),5)

            # print(cv2_image)
            self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warper_image, self.yaw)
            cv2.imshow("slide_img", self.slide_img)
            

            gray_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)

            # low_B = cv2.getTrackbarPos('low_H', 'Simulator_Image')
            # low_G = cv2.getTrackbarPos('low_S', 'Simulator_Image')
            # low_R = cv2.getTrackbarPos('low_V', 'Simulator_Image')
            # high_B = cv2.getTrackbarPos('high_H', 'Simulator_Image')
            # high_G = cv2.getTrackbarPos('high_S', 'Simulator_Image')
            # high_R = cv2.getTrackbarPos('high_V', 'Simulator_Image')
            lower_c = np.array([0, 156, 190]) 
            upper_c = np.array([219, 190, 255])

            s_image = cv2.inRange(cv2_image, lower_c, upper_c)
            # cv2.imshow("s", s_image)
            s_mask = cv2.inRange(s_image, 150, 255)
            blur_curve = cv2.GaussianBlur(s_mask, (ksize, ksize), 0)
            _, s_img = cv2.threshold(blur_curve, 200, 255, cv2.THRESH_BINARY)
            warper_s = self.warper.warp(s_img)

            self.curve_img, self.angle = self.slidewindow.curve(warper_s)
            # print("angle", self.angle)
            cv2.imshow("out", self.curve_img)





            # if self.yaw <= -20 :
            #     self.slide_x_location += 10

            # cv2.imshow("original", cv2_image)
            cv2.waitKey(1)
            # cv2.imshow("warper", warper_s)
            # cv2.waitKey(1)
        
            # print("success")
            # try :
            #     cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)
            #     # cv2_image = self.bridge.imgmsg_to_cv2(_data)
            #     cv2_image = self.bridge.cv2_to_imgmsg(cv2_image, encoding="bgr8")
            #     # self.original_img = cv2_image
            #     cv2.imshow("original", cv2_image)
            #     print("success")
            # except :
            #     print("fail")
            #     pass



            # self.x_location : 현재 차선에서 중앙 값
            # middle x location : 320
        
            self.error_lane = 320 - self.slide_x_location 

            if self.slide_x_location < 270 or self.slide_x_location > 370:
                self.motor_msg = 3
                self.steering_val = self.error_lane * 0.003
            else:
                self.motor_msg = 5 
                self.steering_val = self.error_lane * 0.001
            
            print('angle', abs(self.angle))
            if self.yaw < -20 and abs(self.angle) <= 73 :
                # print("####################")
                # self.motor_msg = 5
                self.steering_val = (90 - abs(self.angle))  * -0.02
            elif self.angle >= 500:
                self.motor_msg = 10

        # self.camera_steering_pub.publish(self.steering_val)

    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):

        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = self.steering_val
        self.ctrl_cmd_msg.accel = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)






def nothing(x):
    pass


if __name__ == "__main__":
    control = Controller()
 

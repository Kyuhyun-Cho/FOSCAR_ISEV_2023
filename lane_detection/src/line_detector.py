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
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd
# from sensor_msg.msg import CompressedImage
from cv_bridge import CvBridge





class Controller() :

    def __init__(self) :
        rospy.init_node("line_detector")


        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)


        self.bridge = CvBridge()
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.original_img = []
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)

        # self.camera_steering_pub= rospy.Publisher('/cam_steer', CtrlCmd, queue_size=1)

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0.0
        self.current_lane_window = ""

        self.initialized = False # image_callback

        self.error_lane = 0
        self.steering_val = 0


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
        
    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude

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
        if self.original_longitude  <= 1 and self.original_latitude <= 1 :

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
            self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warper_image)
            # cv2.imshow("slide_img", self.slide_img)


            # cv2.imshow("original", cv2_image)
            cv2.waitKey(1)
            # cv2.imshow("warper", warper_image)
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
 

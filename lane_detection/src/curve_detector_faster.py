#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from warper import Warper
from curve_slide_window_faster import SlideWindow


### 
# from warper import Warper
# from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import Float64, Bool
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd
# from sensor_msg.msg import CompressedImage
from cv_bridge import CvBridge
from math import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler




class Controller() :

    def __init__(self) :
        rospy.init_node("curve_detector")

        self.bridge = CvBridge()
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.original_img = []
        self.curve_img = None
        self.original_longitude = 0.0
        self.original_latitude = 0.0
        self.yaw = 0.0

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.mission_sub = rospy.Subscriber("/mission", Bool, self.mission_callback)
        
        # self.mission_sub = rospy.Subsriber("/mission")

        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.ImuCB) ## Vehicle Status Subscriber

        self.ctrl_cmd_pub = rospy.Publisher('/curve_cmd', CtrlCmd, queue_size=1)

        # self.camera_steering_pub= rospy.Publisher('/cam_steer', CtrlCmd, queue_size=1)

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0.0
        self.current_lane_window = ""

        self.initialized = False # image_callback

        self.error_lane = 0
        self.steering_val = 0

        self.curve_steering = 0.0
        self.angle = 0.0
        self.cnt = 0
        self.turn_flag = 1
        self.turn_left = 0

        self.mission = False

        # For test
        self.motor_msg = 5.0
        rate = rospy.Rate(20) # 30hz
        while not rospy.is_shutdown():
            # self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
            
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
    def ImuCB(self, msg) :
        # self.odom_msg.pose.pose.orientation.x = msg.orientation.x
        # self.odom_msg.pose.pose.orientation.y = msg.orientation.y
        # self.odom_msg.pose.pose.orientation.z = msg.orientation.z
        # self.odom_msg.pose.pose.orientation.w = msg.orientation.w

        quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        _, _, self.yaw =  euler_from_quaternion(quaternion)
        self.yaw = degrees(self.yaw) 

    def mission_callback(self, msg) :
        self.mission = msg.data
        # print('curve CB', self.mission)
        

    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude
        # print(self.original_longitude, self.original_latitude)

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

        if self.original_longitude  <= 1 and self.original_latitude <= 1 and self.mission == False:
            print('curve')
            cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)
            # cv2.imshow("original", cv2_image)



            # low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
            # low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
            # low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
            # high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
            # high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
            # high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


            # hsv_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
            # lower_lane = np.array([low_H, low_S, low_V]) 
            # upper_lane = np.array([high_H, high_S, high_V])
            # lower_lane = np.array([0, 0, 126]) #0 46 170 / white lane: 0 0 126
            # upper_lane = np.array([255, 64, 180]) #79 255 255 / white lane : 255, 64, 180
            # lane_mask = cv2.inRange(hsv_image, lower_lane, upper_lane)
            ksize = 5
            # blur_lane = cv2.GaussianBlur(lane_mask, (ksize, ksize), 0)
            # _, lane_image = cv2t.threshold(blur_lane, 200, 255, cv2.THRESH_BINARY)
            


            gray_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)


            # low_B = cv2.getTrackbarPos('low_H', 'Simulator_Image')
            # low_G = cv2.getTrackbarPos('low_S', 'Simulator_Image')
            # low_R = cv2.getTrackbarPos('low_V', 'Simulator_Image')
            # high_B = cv2.getTrackbarPos('high_H', 'Simulator_Image')
            # high_G = cv2.getTrackbarPos('high_S', 'Simulator_Image')
            # high_R = cv2.getTrackbarPos('high_V', 'Simulator_Image')
            # lower_c = np.array([low_B, low_G, low_R]) 
            # upper_c = np.array([high_B, high_G, high_R])
            lower_c = np.array([92,114,116])
            upper_c = np.array([214,208,228])
            # cv2.imshow('gray', gray_image)
            curve_image = cv2.inRange(cv2_image, lower_c, upper_c)
            # cv2.imshow('cv2', cv2_image)
            # cv2.imshow("ci", curve_image)
            curve_mask = cv2.inRange(curve_image, 150, 255)
            blur_curve = cv2.GaussianBlur(curve_mask, (ksize, ksize), 0)
            _, curve_img = cv2.threshold(blur_curve, 200, 255, cv2.THRESH_BINARY)
            edges = cv2.Canny(curve_img, 50, 150)
            warper_image = self.warper.warp(curve_img)
            # warper_curve = self.warper.warp(curve_img)




            self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warper_image, self.yaw, self.turn_left)
            self.curve_img, self.angle = self.slidewindow.curve(warper_image)
            # cv2.imshow("curve_slide_img", self.slide_img)
            # cv2.imshow("slide_img", self.slide_img)
            # cv2.imshow("curve", self.curve_img)
            print('angle: ', self.angle)


            # cv2.imshow("original", cv2_image)
            cv2.waitKey(1)
            # cv2.imshow("warper", warper_image)
            # cv2.waitKey(1)

            print(self.yaw)

            self.error_lane = 320 - self.slide_x_location 
            # print("yaw", self.yaw)

            if self.slide_x_location < 270 or self.slide_x_location > 370:
                self.motor_msg = 4
                self.steering_val = self.error_lane * 0.003
                if self.yaw < -80 and self.turn_flag == -1:
                    self.motor_msg = 10
            else:
                self.motor_msg = 5
                self.steering_val = self.error_lane * 0.001
                if self.yaw < -80 and self.turn_flag == -1:
                    self.motor_msg = 15

            if abs(self.angle) >= 300 : 
                print("차선인식 주행 중")

            if abs(self.angle) <= 60 :
                print("커브선 인식: 커브선 각도", self.angle, self.turn_left, self.turn_flag)
                self.motor_msg = 2
                self.steering_val = (90 - abs(self.angle)) * self.turn_flag * 0.02
                if self.turn_flag == -1 :
                    print("우회전 중입니다")
                else :
                    print("좌회전 중입니다.")
            print('yaw', self.yaw)
            if self.turn_flag == 1 and -45 <= self.yaw <= -40 and self.turn_left == 0:
                self.motor_msg = 3
                print("좌회전 어느정도 완료했습니다.")
                self.steering_val = 1
                self.turn_left = 1 # 죄회전을 함
            if self.turn_left == 1 and self.yaw < -16 :
                self.motor_msg = 3
                print('좌회전 완료 후 차량 차선 복귀 ')
                self.steering_val = 1
                # self.turn_left = 2
            elif self.turn_left == 1 and self.yaw >= -17 :
                self.motor_msg = 3
                print('좌회전 종료')
                self.turn_left = 2
                self.turn_flag = -1
            elif self.turn_flag == -1 and self.turn_left == 2 and self.yaw > -85 and abs(self.angle) != 500 :
                self.steering_val = -1
                self.motor_msg = 3
            print(self.steering_val)
                # if self.angle < 0 :
                #     self.steering_val = (90 - abs(self.angle)) * 0.013
                #     self.steering_val *= - 1
                # else :
                #     self.steering_val = (90 - abs(self.angle)) * 0.02
            
            
            # if self.yaw >= -17 and self.turn_flag == 1:
            #     self.motor_msg = 1
            #     self.cnt += 1
            #     if self.cnt >= 5 : 
            #         self.turn_flag = -1

            
        # print("angle :", self.angle, "x_location:", self.slide_x_location)
        else :
            self.cnt = 0
            self.turn_flag = 1
            self.turn_left = 0


    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = self.steering_val
        self.ctrl_cmd_msg.accel = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)






def nothing(x):
    pass


if __name__ == "__main__":
    control = Controller()
 

#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int64MultiArray


# from sensor_msg.msg import CompressedImage
from cv_bridge import CvBridge

class Controller() :

    def __init__(self) :
        rospy.init_node("traffic_light")
        self.bridge = CvBridge()
        self.original_img = []
        self.traffic_pub=rospy.Publisher("/traffic_light", Int64MultiArray, queue_size=1)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)


        self.traffic_signal = 0
        # slide window return variable initialization

        self.initialized = False # image_callback

        # rospy.Subscriber("/image_jpeg/compressed", Image, self.image_callback)
        rospy.spin()
    
    # def runnung(self, _event) :
        

    def image_callback(self, _data) :
        cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)                                                                                  
        cv2_image = cv2_image[100:200, 140:500]
        cv2_image = cv2.resize(cv2_image, (700,700))
        # cv2_image = cv2_image[ 370:410, 140:550 ]
        # red_image=cv2.imread('/home/park/사진/red_light.png')
        # green_image=cv2.imread('/home/park/사진/green_light.png')
        # yellow_image=cv2.imread('/home/park/사진/yellow_light.png')

        # cv2.imshow("original", cv2_image)

        hsv_red = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
        # hsv_yellow = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
        hsv_green = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

        #red mask 1
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        red_mask1 = cv2.inRange(hsv_red, lower_red1, upper_red1)

        #red mask2
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255]) 
        red_mask2 = cv2.inRange(hsv_red, lower_red2, upper_red2)

        lower_green = np.array([50, 70, 70]) #(40,50,50) (60,50,50)
        upper_green = np.array([80, 255, 255]) #70. 90

        mask_red = red_mask1 + red_mask2
        mask_green = cv2.inRange(hsv_green, lower_green, upper_green)

        red_count=np.count_nonzero(mask_red)
        green_count=np.count_nonzero(mask_green)

        # print("red:", np.count_nonzero(mask_red)) #18대~30대
        # print("green:", np.count_nonzero(mask_green)) #직진:50대~60대


        # Default : 0, Red : 1, Green :2
        

        # if red_count >=18 and red_count <=25:
        #     self.traffic_signal = 1
        #     # print("RED LIGHT DETECTED")
        # elif green_count >=50 and green_count <=60:
        #     self.traffic_signal = 2
        #     # print("GREEN LIGHT DETECTED")
        # else: 
        #     self.traffic_signal = 0

        if green_count >=550:
            self.traffic_signal = 2
            # print("GREEN LIGHT DETECTED")
        else: 
            self.traffic_signal = 1
        

        msg = Int64MultiArray()
        msg.data = [red_count, green_count]

        self.traffic_pub.publish(msg)

        res_red = cv2.bitwise_and(hsv_red, cv2_image, mask=mask_red)
        res_green = cv2.bitwise_and(hsv_green, cv2_image, mask=mask_green)

        # cv2.imshow('red', res_yellow)
        # cv2.imshow('green', res_green)

        # lower_yellow = np.array([10, 30, 30])
        # upper_yellow = np.array([30, 255, 255])
        # cv2.imshow('red', res3)        
        # # h, s, v = cv2.split(hsv_image)
        # # h = cv2.inRange(h, 8, 20)

        # lower_lane = np.array([low_H, 0, 0]) 
        # upper_lane = np.array([high_H, 255, 255])
        


        # lane_mask = cv2.inRange(hsv_image, lower_lane, upper_lane)

        # _, lane_image = cv2.threshold(lane_mask, 200, 255, cv2.THRESH_BINARY)

        # cv2.imshow("sign_image", lane_image)

        # orange = cv2.bitwise_and(hsv_image, hsv_image, mask = h)
        # orange = cv2.cvtColor(orange, cv2.COLOR_HSV2BGR)
        # cv2.imshow("orange", orange)
        # cv2.imshow("hsv", hsv_image)

        # cv2.imshow("original", cv2_image)
        cv2.waitKey(1)
        # cv2.imshow("warper", warper_image)
    
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

if __name__ == "__main__":
    control = Controller()
 

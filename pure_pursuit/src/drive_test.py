#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool, String, Int64
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv


from utils import pathReader,findLocalPath,purePursuit
from std_msgs.msg import Int64

import tf
import time
from math import *

warnings.simplefilter(action='ignore', category=FutureWarning)


# 아이오닉 5 -> 조향값(servo_msg) 0일 때 직진 양수이면 좌회전 음수이면 우회전


class Point():
    def __init__(self, min_x, min_y, max_x, max_y):
        self.x1 = min_x
        self.y1 = min_y
        self.x2 = max_x
        self.y2 = max_y

class PurePursuit():
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)


        # self.path_name = 'test_path'
        self.path_name = 'test_path_first'

        # Publisher
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.mission_pub = rospy.Publisher('/mission', String, queue_size = 1)
        self.waypoint_pub = rospy.Publisher('/waypoint', Int64, queue_size = 1)


        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/yaw", Float64, self.yawCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/traffic_light", Int64, self.trafficCB)
        rospy.Subscriber("/cam_steer", CtrlCmd, self.laneCB) # Drive for camera
        rospy.Subscriber("/curve_cmd", CtrlCmd, self.curveCB) # curve Mission
        
        self.steering_angle_to_servo_offset = 0.0 ## servo moter offset
        self.target_x = 0.0
        self.target_y = 0.0

        self.ctrl_cmd_msg = CtrlCmd()
        
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

        self.yaw = 0.0
        self.yaw_rear = False

        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 0.0
        self.accel_msg = 0

        self.traffic_signal = 0


        self.original_longitude = 0
        self.original_latitude = 0


        self.camera_servo_msg = 0.0
        self.camera_motor_msg = 10.0

        self.steering_offset = 0.015

        self.clear_stop_mission = False
        self.clear_start_mission = False

        self.passed_curve = False
        self.curve_servo_msg = 0.0
        self.curve_motor_msg = 0.0

        self.Mission = "line_drive"

        

        ######## For Service ########
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.req = EventInfo()

        self.drive_mode()
        # self.left_light()
        # self.drive_left_light()

        # service request part

        ################################


        # Class
        path_reader = pathReader('path_maker') ## 경로 파일의 위치
        pure_pursuit = purePursuit() ## purePursuit import
        
        # Read path
        self.global_path = path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름

        # Time var
        count = 0
        rate = rospy.Rate(30) # 30hz
                                           
        while not rospy.is_shutdown():

            self.Mission = "line_drive"

            self.accel_msg = 0
            
            
            ## global_path와 WeBot status_msg를 이용해 현재 waypoint와 local_path를 생성
            local_path, current_waypoint, = findLocalPath(self.global_path, self.longitude, self.latitude)

            pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
            pure_pursuit.getEgoStatus(self.longitude, self.latitude, self.yaw) ## pure_pursuit 알고리즘에 WeBot status 적용

            self.steering, self.target_x, self.target_y = pure_pursuit.steering_angle()

            # waypoint 출력
            # print(self.path_name, current_waypoint, len(self.global_path.poses))
            self.waypoint_pub.publish(current_waypoint)


        ################# 일반 주행 조향값 및 속도 설정 ##################################

            # 조향 값 확인 : rostopic echo /sensors/s            # cv2.imshow("curve_slide_img", self.slide_img)
            # cv2.imshow("curve", self.curve_img)


            # cv2.imshow("original", cv2_image)ervo_position_command -> data
            # range : 0.0 ~ 1.0 (straight 0.5)
            self.servo_msg = self.steering*self.steering_offset #+ self.steering_angle_to_servo_offset
            self.motor_msg = 18.0 - min(10, abs(self.servo_msg * 20)) # steering에 따른 속도
            self.brake_msg = 0
            self.ctrl_cmd_msg.longlCmdType = 2


        ################################### 출발 미션 ###################################

            if  current_waypoint <= 45 and self.path_name == 'test_path_first':
                print("START MISSION", current_waypoint)
                self.drive_left_light()
            elif self.path_name == 'test_path_first' and current_waypoint <= 55  and self.clear_start_mission == False:
                self.drive_mode()
                self.clear_start_mission = True

        ################################### 종료 미션 ###################################

            if  730 <= current_waypoint <= 759 and self.path_name == 'test_path_second':
                print("FINISH MISSION", current_waypoint)
                self.drive_right_light()
            elif self.path_name == 'test_path_second' and current_waypoint >= 765 :
                print("parking", current_waypoint)
                # for i in range(1000) :
                self.brake()
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                rospy.sleep(2)
                self.parking()

        ################################### 정지 미션 ###################################

            # 오르막길 일 때는 악셀 밟기
            if 100 <= current_waypoint <= 125 and self.path_name == 'test_path_first' :
                print("Hill accel", current_waypoint)
                # self.ctrl_cmd_msg.longlCmdType = 1
                # self.ctrl_cmd_msg.accel = 1
                # self.ctrl_cmd_msg.acceleration = 1
                self.motor_msg = 18

            if 133 <= current_waypoint <= 136 and self.path_name == 'test_path_first' and self.clear_stop_mission == False:
                self.ctrl_cmd_msg.accel = 0
                self.ctrl_cmd_msg.acceleration = 0
                print("STOP MISSION", current_waypoint)
                self.brake()
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                self.clear_stop_mission = True
                rospy.sleep(3.3) # 3.3초 동안 정지

        ################################### 가속 미션 ################mission####################

            if self.path_name == 'test_path_second' and 400 <= current_waypoint <= 425 :
                self.Mission = "ACCEL_MISSION"
                print("ACCEL MISSION_1", current_waypoint)
                self.motor_msg = 18
                self.ctrl_cmd_msg.longlCmdType = 2
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                continue
            elif self.path_name == 'test_path_second' and 425 <= current_waypoint <= 460 :
                self.Mission = "ACCEL_MISSION"
                print("ACCEL MISSION_2", current_waypoint)
                self.ctrl_cmd_msg.longlCmdType = 1
                self.ctrl_cmd_msg.accel = 1
                self.ctrl_cmd_msg.acceleration = 5
                self.motor_msg = 30
                self.servo_msg /= 5
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                continue
            elif self.path_name == 'test_path_second' and 461 <= current_waypoint <= 490 :
                self.Mission = "ACCEL_MISSION"
                print("ACCEL MISSION_3", current_waypoint)
                self.ctrl_cmd_msg.longlCmdType = 1
                self.ctrl_cmd_msg.accel = 0
                self.ctrl_cmd_msg.acceleration = 0
                self.motor_msg = 17
                self.servo_msg /= 5
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                continue

        ############################# ㄱ자 미션 #########################################
            ### gps blackout 크기 바꿔가면서 주행
            ### 진입할 때 다양한 각도로 진입하면서 주행 테스트 할 것 
            ####### 방법 1 카메라, IMU 사용하여 주행 #####
            if self.path_name == 'test_path_first' :
                self.Mission = "CURVE MISSION"
                if 310 <=current_waypoint <= 350 :
                    # print("CURVE MISSION_START", current_waypoint)
                    self.motor_msg = 5
                if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.passed_curve == False:
                    # print(self.yaw)
                    # print("CURVE MISSION")
                    self.servo_msg = self.curve_servo_msg
                    self.motor_msg = self.curve_motor_msg
                if 500 <= current_waypoint <= 503:
                    self.passed_curve = True

        ############################# S자 미션  ##########################################
            ####### 방법 1 카메라를 사용하여 주행 #####

            ####### s자 미션 진입식 속도 줄이기 #######
            if self.path_name == 'test_path_first' and self.passed_curve == True:
                if 680 <=current_waypoint <= 830 :
                    print("S MISSION_START", current_waypoint)
                    self.motor_msg = 6

            ####### gps 안 들어 올때 카메라로 계산한 steering으로 주행 ##################
                if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and current_waypoint > 530 :
                    self.Mission = "S_MISSION"
                    print("S_MISSION", current_waypoint)
                    self.servo_msg = self.camera_servo_msg
                    self.motor_msg = 15 - min(10, abs(self.camera_servo_msg*20))
                
                # print(self.camera_servo_msg, self.camera_motor_msg)

            ####### 방법 2 x  #######
        

            ###### 방법 3 Lidar 센서, camera 센서 활용 Lidar로 보정


        #################### T자 미션 전용 조향값 및 속도 설정 #############################

            # 후진 할 때는 조향값 반대, 속도는 느리게
            if self.path_name == 'test_path_1' or self.path_name == 'test_path_2' or self.path_name == 'test_path_3' :
                print("T MISSION", current_waypoint)
                self.Mission = "T MISSION"
                self.steering_offset = 0.03
                self.motor_msg = 5
                if (self.path_name == 'test_path_1' and current_waypoint + 70 >= len(self.global_path.poses)) or (self.path_name == 'test_path_3' and current_waypoint <= 100) : 
                    self.motor_msg = 1
            else :
                self.steering_offset = 0.015

            if self.yaw_rear == True :
                self.steering, self.target_x, self.target_y = pure_pursuit.steering_angle()
                self.servo_msg = self.steering*self.steering_offset 
                self.servo_msg *= -1
                self.motor_msg = 1


        ############################## 방법 1 Path switching   #####################################


            # print(current_waypoint, len(self.global_path.poses))
            # T자 주차를 위한 path switching
            if current_waypoint + 5  >= len(self.global_path.poses) :
                if self.path_name == 'test_path_first' :
                    self.path_name = 'test_path_1'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.ctrl_cmd_msg.longlCmdType = 1
                    self.is_swith_path = False
                    print(self.path_name)
                elif self.path_name == 'test_path_1' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'test_path_2'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    self.is_swith_path = False
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(1)
                    self.rear_mode()
                    print(self.path_name)
                elif self.path_name == 'test_path_2' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'test_path_3'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    self.is_swith_path = False
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(1)
                    self.drive_mode()
                    print(self.path_name)
                elif self.path_name == 'test_path_3' : 
                    self.path_name = 'test_path_second'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.is_swith_path = False
    

        #################################### 방법 2 후방 카메라를 통한 주차 ###############################
            
            # print(current_waypoint)
                
        ################################ 신호등 ##########################################################

            # 빨간불 -> 정지
            # 구간 출력을 위한 조건문  ##############################################
            if (self.path_name == 'test_path_second' and 70 <= current_waypoint <= 130) or (self.path_name == 'test_path_first' and (535 <= current_waypoint <= 542 or 965 <= current_waypoint <= 971)) :
                print("Traffic Mission", current_waypoint)
            #######################################################################

            # 첫번째 신호등
            if self.path_name == 'test_path_first' and 535 <= current_waypoint <= 542 and self.traffic_signal == 1:
                print("RED_SIGNAL", current_waypoint)
                self.brake()
            # 두번째 신호등
            if self.path_name == 'test_path_first' and 965 <= current_waypoint <= 971 and self.traffic_signal == 1:
                print("RED_SIGNAL", current_waypoint)
                self.brake()
            # 세번째 신호등
            if self.path_name == 'test_path_second' and 70 <= current_waypoint <= 130 and self.traffic_signal == 1:
                print("RED_SIGNAL", current_waypoint)
                self.brake()
            # 초록불 좌회전 깜빡이 켜기
            if self.path_name == 'test_path_second' and 70 <= current_waypoint <= 130:
                print("TURN LEFT", current_waypoint)
                self.drive_left_light()
            # 깜빡이 끄기
            if self.path_name == 'test_path_second' and 130 < current_waypoint <= 150 :
                self.drive_mode()


            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            self.mission_pub.publish(self.Mission)
            
            # global path 출력
            if count==300 : 
                self.global_path_pub.publish(self.global_path)
                count=0
            count+=1


            
            rate.sleep()



######################      Service Request     #############################

    # option - 1 : ctrl_mode / 2 : gear / 4 : lamps / 6 : gear + lamps
    # gear - 1: P / 2 : R / 3 : N / 4 : D

    def drive_mode(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 0
        response = self.req_service(self.req)
        self.yaw_rear = False

    def rear_mode(self):
        self.req.option = 2
        self.req.gear = 2
        response = self.req_service(self.req)
        self.yaw_rear = True
        # self.req.lamps
    
    def drive_left_light(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 1
        response = self.req_service(self.req)
    
    def drive_right_light(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 2
        response = self.req_service(self.req)

    def emergency_mode(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.emergencySignal = 1
        response = self.req_service(self.req)

    def parking(self) :
        self.req.option = 6
        self.req.gear = 1
        self.req.lamps.turnSignal = 0
        response = self.req_service(self.req)

    def brake(self) :
        self.ctrl_cmd_msg.longlCmdType = 1
        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 1



    
################################################################################
        


    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude
        proj_UTMK = Proj(init='epsg:5179')
        proj_WGS84 = Proj(init='epsg:4326')
        self.longitude, self.latitude = transform(proj_WGS84, proj_UTMK, msg.longitude, msg.latitude) 
        
        #WGS84
        # self.longitude = msg.longitude
        # self.latitude = msg.latitude
    
    def yawCB(self, msg):
        self.yaw = msg.data
        if self.yaw_rear == True :
            self.yaw += 180
        # print(self.yaw)

    def trafficCB(self, msg):
        # Default : 0, Red : 1, Green :2
        self.traffic_signal=msg.data
        # print(self.traffic_signal)

    def laneCB(self, msg) : 
        # print(msg)
        self.camera_servo_msg = msg.steering
        self.camera_motor_msg = msg.velocity

    def curveCB(self, msg) :
        self.curve_servo_msg = msg.steering
        self.curve_motor_msg = msg.velocity


        

    def isMissionArea(self, x1, y1, x2, y2):
        if (x1 <= self.status_msg.position.x <= x2) and (y1 <= self.status_msg.position.y <= y2):
            return True
        else:
            return False


    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = servo_msg
        self.ctrl_cmd_msg.brake = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    
    def change_path_1(self) :
        self.path_name = 'test_path_1'
        self.rear_mode()


    def change_path_2(self) :
        self.path_name = 'test_path_2'

    def change_path_3(self) :
        self.path_name = 'test_path_3'


if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass
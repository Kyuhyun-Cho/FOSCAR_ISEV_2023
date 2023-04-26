#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool, String, Int64MultiArray, Float64MultiArray, Int64
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

DEFAULT_LFD = 18

class Point():
    def __init__(self, min_x, min_y, max_x, max_y):
        self.x1 = min_x
        self.y1 = min_y
        self.x2 = max_x
        self.y2 = max_y

class PurePursuit():
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        self.path_name = 'final_path_first'
        # self.path_name = 'final_path_second'


        # Publisher
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.mission_pub = rospy.Publisher('/mission', String, queue_size = 1)
        self.waypoint_pub = rospy.Publisher('/waypoint', Int64, queue_size = 1)


        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/yaw", Float64, self.yawCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/traffic_light", Int64MultiArray, self.trafficCB)
        rospy.Subscriber("/cam_steer", CtrlCmd, self.laneCB) # Drive for camera
        rospy.Subscriber("/curve_cmd", CtrlCmd, self.curveCB) # curve Mission
        rospy.Subscriber("/dy_obs_info", Float64MultiArray, self.dyObsCB) # Drive for camera

        
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
        # self.passed_curve = True

        self.dynamic_flag = False
        self.dynamic_done = False

        self.curve_servo_msg = 0.0
        self.curve_motor_msg = 0.0

        self.Mission = "line_drive"

        self.green_count = 0
        self.red_count = 0

        self.dy_obs_info = [0, 0, 0, 0]

        self.T_mission = False
        

        ######## For Service ########
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.req = EventInfo()

        self.forward_mode()
        # self.left_light()
        # self.drive_left_signal()

        # service request part

        ################################


        # Class
        path_reader = pathReader('path_maker') ## 경로 파일의 위치
        self.pure_pursuit = purePursuit() ## purePursuit import
        
        # Read path
        self.global_path = path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름

        # Time var
        count = 0
        rate = rospy.Rate(30) # 30hz
                                           
        while not rospy.is_shutdown():
            
            self.Mission = "line_drive"

            self.accel_msg = 0

            # global path 출력    
            self.global_path_pub.publish(self.global_path)
            
            ## global_path와 WeBot status_msg를 이용해 현재 waypoint와 local_path를 생성
            local_path, current_waypoint, = findLocalPath(self.global_path, self.longitude, self.latitude)

            print("Current Waypoint: ", current_waypoint)
           
            self.pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용

            if self.T_mission == False : 
                self.pure_pursuit.getEgoStatus(self.longitude, self.latitude, self.yaw, self.motor_msg) 
            else :
                self.pure_pursuit.getEgoStatus(self.longitude, self.latitude, self.yaw, 5/2 +0.5)

            self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(DEFAULT_LFD)

            # self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle()

            # waypoint 출력
            # print(self.path_name, current_waypoint, len(self.global_path.poses))
            self.waypoint_pub.publish(current_waypoint)
            self.mission_pub.publish(self.Mission)

            # print(self.passed_curve)w

        ############################ 일반 주행 조향값 및 속도 설정 ##################################

            # 조향 값 확인 : rostopic echo /sensors/s            # cv2.imshow("curve_slide_img", self.slide_img)
            # cv2.imshow("curve", self.curve_img)


            # cv2.imshow("original", cv2_image)ervo_position_command -> data
            # range : 0.0 ~ 1.0 (straight 0.5)

            self.ctrl_cmd_msg.longlCmdType = 2
            self.servo_msg = self.steering*self.steering_offset #+ self.steering_angle_to_servo_offset
            # self.motor_msg = 18.0 - min(12, abs(self.servo_msg * 25)) # steering에 따른 속도
            self.motor_msg = 18.0
            self.brake_msg = 0
         


        ###################################################################### 출발 미션 ###################################################################### 
            if self.path_name == 'final_path_first':
                if  current_waypoint <= 70:
                    self.drive_left_signal()
                    continue
                elif current_waypoint <= 80  and self.clear_start_mission == False:
                    self.forward_mode()
                    self.clear_start_mission = True


        ###################################################################### 오르막길 정지 미션 ######################################################################
            if self.path_name == 'final_path_first':
                if 145 <= current_waypoint <= 199: # 오르막길 엑셀 밟기
                    self.setMotorMsgWithVel(18)

                if 191 <= current_waypoint <= 196 and self.clear_stop_mission == False:
                    self.ctrl_cmd_msg.accel = 0
                    self.ctrl_cmd_msg.acceleration = 0
                    self.brake()

                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    self.clear_stop_mission = True
                    rospy.sleep(3.3) # 3.3초 동안 정지
                    continue


        ###################################################################### 직각 코스 미션 ######################################################################
            if self.path_name == 'final_path_first':
                if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.passed_curve == False:
                    self.motor_msg = self.curve_motor_msg
                    self.servo_msg = self.curve_servo_msg
                    print("CURVE")


                if 305 <= current_waypoint <= 380: # Curve Slow Down
                    print("CURVE_START")
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)
                    

                elif 380 < current_waypoint <= 414 :  # 직각 코스 진입 Slow Down
                    print("CURVE_SLOW")
                    self.setMotorMsgWithVel(5)
                    self.setServoMsgWithLfd(3)
                
                    
                if 554 <= current_waypoint <=  602 : # 직각 코스 종료 Slow Down
                    print("FINSH_CURVE")
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)

                if 597 <= current_waypoint <= 615:
                    self.passed_curve = True


        ###################################################################### S자 코스 미션  ######################################################################
            if self.path_name == 'final_path_first' and self.passed_curve == True:
                if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.passed_curve : # GPS 음영구역 진입 시 Camera Steering으로 주행
                    self.setMotorMsgWithVel(15 - min(7, abs(self.camera_servo_msg*20)))
                    self.servo_msg = self.camera_servo_msg
                    # if self.yaw < -40 :
                    #     self.setMotorMsgWithVel(3)
                    #     # self.servo_msg *= 2
                

                if 720 <= current_waypoint <= 780 : # S자 코스 진입 Slow down
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)
                    
                elif 780 <current_waypoint <= 845 : # S자 코스 진입 Slow down
                    self.setMotorMsgWithVel(6)
                    self.setServoMsgWithLfd(3)

            
            ####### s자 끝나고 차선 복귀할 때까지는 천천히 달리기 ########################
                if 960 <= current_waypoint <=  1010 :
                    self.setMotorMsgWithVel(6)
                    if -80 < self.yaw < -10 :
                        self.servo_msg = self.camera_servo_msg
                    self.Mission = "S_MISSION"
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    print("S_MISSION_FINSH")
                    continue



       
                        

        ###################################################################### T자 주차 미션  ######################################################################

            # 후진 할 때는 조향값 반대, 속도는 느리게
            if (self.path_name == 'test_path_1' or 
                self.path_name == 'test_path_2' or 
                self.path_name == 'test_path_3'):

                self.setMotorMsgWithVel(5)
                self.setServoMsgWithLfd(5)
                self.servo_msg *= 2

                self.T_mission = True
                if (self.path_name == 'test_path_1' and current_waypoint + 70 >= len(self.global_path.poses)) or (self.path_name == 'test_path_3' and current_waypoint <= 100) : 
                    self.setMotorMsgWithVel(2)
                    self.setServoMsgWithLfd(5)
                    self.servo_msg *= 2
                elif  (self.path_name == 'test_path_3' and current_waypoint >= 250) :
                    self.setMotorMsgWithVel(10)
                    self.setServoMsgWithLfd(5)
                    self.servo_msg *= 2
            else:
                self.T_mission = False

            if self.yaw_rear == True :
                self.setMotorMsgWithVel(1)
                self.setServoMsgWithLfd(1)
                self.servo_msg *= -1


            # T자 주차를 위한 path switching
            if current_waypoint + 5  >= len(self.global_path.poses) :
                if self.path_name == 'final_path_first':
                    self.path_name = 'test_path_1'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.ctrl_cmd_msg.longlCmdType = 1
                    self.is_swith_path = False
                    # print(self.path_name)
                elif self.path_name == 'test_path_1' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'test_path_2'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    self.is_swith_path = False
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(2)
                    self.rear_mode()
                    # print(self.path_name)
                    continue
                elif self.path_name == 'test_path_2' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'test_path_2-1'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.is_swith_path = False
                elif self.path_name == 'test_path_2-1' and current_waypoint +1 >= len(self.global_path.poses): 
                    self.path_name = 'test_path_3'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    self.is_swith_path = False
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(2)
                    self.forward_mode()
                    # print(self.path_name)
                elif self.path_name == 'test_path_3' and current_waypoint +1 >= len(self.global_path.poses): 
  
                    self.path_name = 'final_path_second'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.is_swith_path = False
    

            ###################################################################### 가속 미션 ######################################################################
            if self.path_name == 'final_path_second':
                if 430 <= current_waypoint <= 440 :
                    self.setMotorMsgWithVel(12+min(current_waypoint-430,6))
                    self.setServoMsgWithLfd(current_waypoint-427)
                    # self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    # continue
                elif 441 <= current_waypoint <= 460 :
                    self.setMotorMsgWithVel(19)
                    self.setServoMsgWithLfd(min(current_waypoint-428,18))
                    # self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    # continue

                # elif 461 <= current_waypoint <= 480 :
                #     # self.ctrl_cmd_msg.longlCmdType = 1
                #     # self.ctrl_cmd_msg.accel = 1
                #     # self.motor_msg = 30
                #     # # self.servo_msg /= 5
                #     self.setMotorMsgWithVel(25)
                #     self.setServoMsgWithLfd(30)
                #     self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                #     continue

                elif 461 <= current_waypoint <= 545: # 481 -> 461
                    self.setMotorMsgWithVel(30)
                    self.setServoMsgWithLfd(25)
                    self.servo_msg /= 4
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                elif 546 <= current_waypoint <= 566:
                    # self.ctrl_cmd_msg.longlCmdType = 1
                    # self.ctrl_cmd_msg.accel = 0
                    # self.ctrl_cmd_msg.acceleration = 0
                    # self.motor_msg = 13
                    # self.servo_msg /= 5
                    self.setMotorMsgWithVel(18)
                    self.setServoMsgWithLfd(25)
                    self.setBrakeMsgWithNum(0.7)
                    self.servo_msg /= 3
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue
                
                else:
                    self.setMotorMsgWithVel(18)
                    self.setServoMsgWithLfd(20)
                    self.setBrakeMsgWithNum(0.0)

            # self.forward_mode()

            ###################################################################### 종료 미션 ######################################################################

            if self.path_name == 'final_path_second':
                if  780 <= current_waypoint <= 820 :
                    self.drive_right_signal()

                elif 800 <= current_waypoint:
                    self.brake()
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(2)
                    self.parking()
                    continue


            ###################################################################### 곡선 코스 미션  ######################################################################
            if self.path_name == 'final_path_first':
                if (1260 <= current_waypoint <= 1305 or 
                    1335 <= current_waypoint <= 1410):
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)
            
            elif self.path_name == 'final_path_second':
                if (86 <= current_waypoint <= 128 or
                    265 <= current_waypoint <= 325):
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)

                elif (129 <= current_waypoint <= 200):
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(min(18, current_waypoint-132))

                elif (370 <= current_waypoint <= 430 or
                      630 <= current_waypoint <= 710):
                    self.setMotorMsgWithVel(12)
                    self.setServoMsgWithLfd(3)

            ##################################################################### 동적 장애물 미션 ######################################################################
            if self.path_name == 'final_path_first':
                if (1020 <= current_waypoint <= 1080 or 
                    1205 <= current_waypoint <= 1277 or 
                    1375 <= current_waypoint <= 1460):

                    if (0 < self.dy_obs_info[0] < 5.5 and 2.6 >= abs(self.dy_obs_info[1]) and not self.dynamic_done): # Dynamic Obstacle Detected
                        print("BRAKE")
                        self.brake()
                        self.emergency_mode()
                        self.dynamic_flag = True
                        continue

                    if (self.dynamic_flag == True):
                        self.dynamic_done = True

                    if (self.dynamic_done == False):
                        print("SLOW DOWN")
                        self.setMotorMsgWithVel(8)
                        self.forward_mode()

                    else:
                        self.setMotorMsgWithVel(18)
                        self.forward_mode()
 

            elif self.path_name == 'final_path_second': 
                if (170 <= current_waypoint <= 220 or 
                    580 <= current_waypoint <= 629):

                    if (0 < self.dy_obs_info[0] < 5.5 and 2.6 >= abs(self.dy_obs_info[1]) and not self.dynamic_done): # Dynamic Obstacle Detected
                        print("BRAKE")
                        self.brake()
                        self.emergency_mode()
                        self.dynamic_flag = True
                        continue

                    if (self.dynamic_flag == True):
                        self.dynamic_done = True

                    if (self.dynamic_flag == False):
                        print("SLOW DOWN")
                        self.setMotorMsgWithVel(8)
                        self.forward_mode()

                    else:
                        self.setMotorMsgWithVel(18)
                        self.forward_mode()
  

            ###################################################################### 신호등 미션 ######################################################################
            # 빨간불 -> 정지
            # 구간 출력을 위한 조건문  
            # if (self.path_name == 'final_path_second' and 70 <= current_waypoint <= 130) or (self.path_name == 'final_path_first' and (535 <= current_waypoint <= 542 or 965 <= current_waypoint <= 971)) :
            #     print("Traffic Mission", current_waypoint)
            ########################################################################################################################################################
            
            if self.path_name == 'final_path_first':
                if 629 <= current_waypoint <= 636 and self.green_count - self.red_count <500: # 첫번째 신호등
                    self.brake()
            
                if self.path_name == 'final_path_first' and 1088 <= current_waypoint <= 1095 and self.green_count - self.red_count <500: # 두번째 신호등
                    self.brake()
            

            if self.path_name == 'final_path_second': 
                if 81 <= current_waypoint <= 89 and (self.green_count - self.red_count < 600 or self.red_count > 150) : # 세번째 신호등
                    self.brake()
            
                # if 81 <= current_waypoint <= 157: # 네번째 신호등
                #     self.drive_left_signal()

                # if 158 < current_waypoint <= 168 : # 다섯번째 신호등
                #     self.forward_mode()




            ########################################################################################################################################################
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            # self.forward_mode()
            rate.sleep()
            ########################################################################################################################################################

###################################################################### Service Request  ######################################################################
    # option - 1 : ctrl_mode / 2 : gear / 4 : lamps / 6 : gear + lamps
    # gear - 1: P / 2 : R / 3 : N / 4 : D
##############################################################################################################################################################

    def forward_mode(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 0
        self.req.lamps.emergencySignal = 0
        response = self.req_service(self.req)
        self.yaw_rear = False
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def rear_mode(self):
        self.req.option = 2
        self.req.gear = 2
        response = self.req_service(self.req)
        self.yaw_rear = True
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def drive_left_signal(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 1
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def drive_right_signal(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 2
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def emergency_mode(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.emergencySignal = 1
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def parking(self) :
        self.req.option = 6
        self.req.gear = 1
        self.req.lamps.turnSignal = 0
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def brake(self) :
        self.ctrl_cmd_msg.longlCmdType = 2
        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 1.0
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)



    
###################################################################### Call Back ######################################################################

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
        self.green_count=msg.data[1]
        self.red_count=msg.data[0]
        # print(msg.data)
        # print("green", self.green_count)
        # print("red", self.red_count)


    def laneCB(self, msg) : 
        # print(msg)
        self.camera_servo_msg = msg.steering
        self.camera_motor_msg = msg.velocity

    def curveCB(self, msg) :
        self.curve_servo_msg = msg.steering
        self.curve_motor_msg = msg.velocity

    def dyObsCB(self, msg):
        self.dy_obs_info = msg.data



###################################################################### Function ######################################################################

    # def isMissionArea(self, longitude_1, latitude_1, longitude_2, latitude_2):
    #     if (longitude_1 > longitude_2):

    #     if (longitude_1 <= self.longitude <= longitude_2) and (y1 <= self.latitude <= y2):
    #         return True
    #     else:
    #         return False


    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = servo_msg
        self.ctrl_cmd_msg.brake = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)


    def setMotorMsgWithVel(self, velocity):
        self.motor_msg = velocity


    def setServoMsgWithLfd(self, lfd):
        self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(lfd)
        self.servo_msg = self.steering*self.steering_offset

    def setBrakeMsgWithNum(self, brake):
        self.brake_msg = brake

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
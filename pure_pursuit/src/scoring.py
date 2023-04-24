#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import rospy
import os
from morai_msgs.msg import CollisionData
from std_msgs.msg import  String, Int64
warnings.simplefilter(action='ignore', category=FutureWarning)

### wirte Collision Data

class Scoring():
    def __init__(self):
        rospy.init_node('scoring', anonymous=True)
        rospy.Subscriber("/CollisionData", CollisionData, self.CollisionCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("mission", String, self.missionCB)
        rospy.Subscriber('waypoint', Int64, self.waypointCB)
        rate = rospy.Rate(30) # 30hz
        self.Collision_cnt = 0
        self.waypoint = 0
        self.Collision_data = []
        self.score_txt = open('/home/park/ISEV_2023/score.txt', 'a')
        self.mission = "line"

        rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            self.score_txt = open('/home/park/ISEV_2023/score.txt', 'a')
            rate.sleep()
        self.score_txt.close()

    def CollisionCB(self, data) :
        if len(data.collision_object) > 0 :
            self.Collision_cnt += 1
            self.score_txt.write("Mission: "+str(self.mission)[5:]+'\n')
            self.score_txt.write("Waypoint: "+str(self.waypoint)[5:]+'\n')
            self.score_txt.write(str(self.Collision_cnt)+'\n')
            # self.score_txt.write(data.collision_object[0].name +'\n')
            print("----------------Hit----------------")
            print(data.collision_object[0].name)

    def missionCB(self, data) :
        self.mission = data

    def waypointCB(self, data) :
        self.waypoint = data
if __name__ == '__main__':
    try:
        pure_pursuit_= Scoring()
    except rospy.ROSInterruptException:
        pass
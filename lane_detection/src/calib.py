#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
import rospkg
import os
import json
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2 



def translationMTX(x,y,z,) :

    # 이동 행렬(3차원 이동)
    mat = np.array([[1,0,0,x],
                    [0,1,0,y],
                    [0,0,1,z],
                    [0,0,0,1],
                    ])
    return mat

def rotationMTX(yaw, pitch, roll) :
    
    # 회전 변환
    rx = np.array([[1,0,0,0],
                    [0,math.cos(roll),-math.sin(roll),0],
                    [0,math.sin(roll),math.cos(roll),0],
                    [0,0,0,1],
                    ])
    ry = np.array([[math.cos(pitch),0,math.sin(pitch),0],
                    [0,1,0,0],
                    [-math.sin(pitch),0,math.cos(pitch),0],
                    [0,0,0,1],
                    ])
    rz = np.array([[math.cos(yaw),-math.sin(yaw),0,0],
                    [math.sin(yaw),math.cos(yaw),100],
                    [0,0,1,0],
                    [0,0,0,1],
                    ])
    
    R = np.matmul(rx, np.matmul(ry,rz))

    return R

def transformMTX_lidar2cam(params_lidar, params_cam) :

    # position -> 파라미터를 넣음
    lidar_pos = {params_lidar.get(i) for i in (["X", "Y",'"Z"'])}
    cam_pos =  {params_cam.get(i) for i in (["X", "Y",'"Z"'])}


    # 상대 위치를 구함
    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]   

    # 시계방향으로 90도 돌린다음에 z축을 기준으로 돌림
    # 라이다 -> 카메라로 좌표변환 
    RT = np.matmul(translationMTX(x_rel, y_rel, z_rel), rotationMTX(np.deg2rad(-90.),0.,0.))
    RT = np.matmul(RT, rotationMTX(0, 0., np.deg2rad(-90.)))

    RT = np.linalg.inv(RT)

    return RT


def project2img_mtx(params_cam) :

    # focal lengths
    fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    # center of image
    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2

    # transformation matrix from 3d to 2d
    R_f = np.array([[fc_x,0,cx],
                   [0, fc_y, cy]])
    
    return R_f

def drat_pts_img(img, x, y) :
    point_np = img

    # Left lane
    for ctr in zip(x,y) :
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0), -1)

    return point_np

class LIDAR2CAMTransform :
    def __init__(self, params_cam, params_lidar) :
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.RT= transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p) :
        
        xyz_c = np.matmul(np.concatenate([xyz_p.shape[0], np.ones((xyz_p.shape[0] , 1))], axis = 1), self.RT.T)

        return xyz_c
    
    def project_pts2img(self, xyz_c, crop = True) :
        
        xyz_c = xyz_c.T

        xc, yc, zc = xyz_c[0,:].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis = 0))

        xyi = xyi[0:2,:].T

        if crop :
            xyi = self.crop_pts(xyi)
        else :
            pass

        return xyi
    
    ### 포인트 클라우드의 영역이 이미지 안에 다 들어가지 않으므로 crop를 잘하자
    
    def crop_pts(self, xyi) :
        
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:,0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:,1]<self.height), :]

        return xyi
    

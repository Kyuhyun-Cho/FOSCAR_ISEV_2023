#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import numpy as np

class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)
         
        # distort src(INPUT) to dst(OUTPUT) 

        # cv2.circle(cv2_image, (630,470),5,(255,0,0),5) #w h
        # cv2.circle(cv2_image, (480,250),5,(0,0,255),5)

        # cv2.circle(cv2_image, (10,470),5,(255,255,0),5)
        # cv2.circle(cv2_image, (160,250),5,(0,255,255),5)

        src = np.float32([ # 4개의 원본 좌표 점
            [0, 450],
            [160, 300],
            [480, 300],
            [640, 450]
        ])

        dst = np.float32([ # 4개의 결과 좌표 점
            [160, h], # [416, 470.4] # 좌하
            [160, 300], # [224, 470.4] # 좌상
            [480,300], # [-192, 0] # 우상
            [480,h] # [832, 0] # 우하
        ])
        

        self.M = cv2.getPerspectiveTransform(src, dst) # self.M : 투시변환 행렬(src to dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src) # self.Minv : 투시변환 행렬(dst to src)

    def warp(self, img): 
        return cv2.warpPerspective(
            img,
            self.M, 
            (img.shape[1], img.shape[0]), # img w, h
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )
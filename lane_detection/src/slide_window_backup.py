import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *
from matplotlib.pyplot import *
import math
# float로 조향값 public

TOTAL_CNT = 50

class SlideWindow:


    
    def __init__(self):

        # Publisher


        self.current_line = "DEFAULT"
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None
        self.left_cnt = 25
        self.right_cnt = 25

        self.x_previous = 320



    def curve(self, img) :
        ## 수평선 상자
        curve_img = img[360:470, 240:320] # 358
        edges = cv2.Canny(curve_img, 50, 150)
        # lines = cv2.HoughLines(edges,1,np.pi/180, 100)
        rho = 1
        theta = np.pi/180
        threshold = 20
        min_line_length = 20
        max_line_gap = 10000
        max_len = 20
        angle = 500
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
        try :
            for line in lines:
                # print("@#")
                x1, y1, x2, y2 = line[0]
                length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                # print(length)
                if length > max_len:
                    max_len = length
                    longest_line = line
            # print("###")
            # print(longest_line)
            x1, y1, x2, y2 = longest_line[0]
            # print(x1,y1,x2,y2)
            angle = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi
        except :
            pass

        return curve_img, angle



    def slidewindow(self, img, yaw):

        x_location = 320.0
        # init out_img, height, width        
        out_img = np.dstack((img, img, img)) * 255 # deleted
        # out_img = img # added 
        height = img.shape[0]
        width = img.shape[1]
        yaw = yaw

        # num of windows and init the height
        window_height = 15 # 7
        nwindows = 15 # 30
        
        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y 
        nonzero = img.nonzero()
        #print nonzero 
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        #print nonzerox
        # init data need to sliding windows
        margin = 20 
        minpix = 0 #10
        left_lane_inds = []
        right_lane_inds = []

        win_h1 = 370
        win_h2 = 480
        win_l_w_l = 100
        win_l_w_r = 280
        win_r_w_l = 400
        win_r_w_r = 580
        
        # first location and segmenation location finder
        # draw line
        # 130 -> 150 -> 180
        pts_left = np.array([[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]], np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)
        pts_right = np.array([[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]], np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)

        pts_catch = np.array([[0, 340], [width, 340]], np.int32)
        cv2.polylines(out_img, [pts_catch], False, (0,120,120), 1)


        # indicies before start line(the region of pts_left)
        # 337 -> 310
        good_left_inds = ((nonzerox >= win_l_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_l_w_r)).nonzero()[0]
        good_right_inds = ((nonzerox >= win_r_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_r_w_r)).nonzero()[0]

        # left line exist, lefty current init
        y_current = None
        x_current = None
        good_center_inds = None
        p_cut = None

        # check the minpix before left start line
        # if minpix is enough on left, draw left, then draw right depends on left
        # else draw right, then draw left depends on right

        # print(len(good_left_inds), len(good_right_inds))
        print( yaw)
        if yaw <= -20 and len(good_right_inds) >= 50 :
            self.current_line = "RIGHT"
            line_flag = 2
            # x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            x_current = int(np.mean(nonzerox[good_right_inds]))
            y_current = int(np.max(nonzeroy[good_right_inds]))
        elif len(good_left_inds) > len(good_right_inds): 
            # print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')          
            self.current_line = "LEFT"
            line_flag = 1
            x_current = int(np.mean(nonzerox[good_left_inds]))
            y_current = int(np.mean(nonzeroy[good_left_inds]))
            max_y = y_current
        elif len(good_right_inds) > len(good_left_inds):
            # if (self.left_cnt + self.right_cnt <= TOTAL_CNT) :
            #     self.right_cnt += 1
            #     self.left_cnt -=1
            self.current_line = "RIGHT"
            line_flag = 2
            # x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            x_current = int(np.mean(nonzerox[good_right_inds]))
            y_current = int(np.max(nonzeroy[good_right_inds]))
        else:
            print("Cant SEE!!!!")
            self.current_line = "MID"
            line_flag = 3   



        # it's just for visualization of the valid inds in the region: ind dot
        if line_flag == 1:
            for i in range(len(good_left_inds)):
                    img = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0,255,0), -1)
        elif line_flag == 2:
            for i in range(len(good_right_inds)):
                    img = cv2.circle(out_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (255,0,0), -1)
            
        # window sliding and draw
        for window in range(0, nwindows):
            if line_flag == 1: 
                # rectangle x,y range init
                win_y_low = y_current - (window + 1) * window_height
                win_y_high = y_current - (window) * window_height
                win_x_low = x_current - margin
                win_x_high = x_current + margin
                # draw rectangle
                # 0.33 is for width of the road
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                cv2.rectangle(out_img, (win_x_low + int(width * 0.48), win_y_low), (win_x_high + int(width * 0.48), win_y_high), (255, 0, 0), 1)
                # indicies of dots in nonzerox in one square
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                # check num of indicies in square and put next location to current 
                if len(good_left_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_left_inds]))
                
                elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                    p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
                    x_current = int(np.polyval(p_left, win_y_high))
                # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                # print("win:", win_y_low)                    
                if win_y_low >= 338 and win_y_low < 348:
                    # print("x: ", x_current)
                # 0.165 is the half of the road(0.33)
                    x_location = x_current + int(width * 0.24) + 7
                    self.x_previous = x_location
                    cv2.circle(out_img, (x_location, 340), 10, (0, 0, 255), 5)

            elif line_flag == 2: # change line from left to right above(if)
                win_y_low = y_current - (window + 1) * window_height
                win_y_high = y_current - (window) * window_height
                win_x_low = x_current - margin
                win_x_high = x_current + margin
                cv2.rectangle(out_img, (win_x_low - int(width * 0.48), win_y_low), (win_x_high - int(width * 0.48), win_y_high), (0, 255, 0), 1)
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]

                if len(good_right_inds) > minpix:
                    x_current = int(np.mean(nonzerox[good_right_inds]))

                elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                    p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2) 
                    x_current = int(np.polyval(p_right, win_y_high))
                if win_y_low >= 338 and win_y_low < 348:
                    # print("x: ", x_current)
                # 0.165 is the half of the road(0.33)
                    x_location = x_current - int(width * 0.24) -7 
                    self.x_previous = x_location
                    cv2.circle(out_img, (x_location, 340), 10, (0, 0, 255), 5)

            else : # can't see
                print("Cant SEE!!!!")
                x_location = self.x_previous
                cv2.circle(out_img, (x_location, 340), 10, (0, 0, 255), 5)


            if x_location == 320:
                x_location = self.x_previous
                cv2.circle(out_img, (x_location, 340), 10, (0, 0, 255), 5)

            left_lane_inds.extend(good_left_inds)


        return out_img, x_location, self.current_line

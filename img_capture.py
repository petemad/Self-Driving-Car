#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 29 03:17:25 2019

@author: amr
"""

import requests
import sys
import cv2
import time
import numpy as np
import logging
import math
import serial
import copy
from firebase import firebase
firebase = firebase.FirebaseApplication('https://automatic-car-fc178.firebaseio.com/', None)
#result = firebase.get('/task3/data', None)
#print(result)
#url = "http://192.168.43.1:8080/shot.jpg" #Hotspot
#url = "http://10.104.179.88:8080/shot.jpg" #Mobile data
#url = "http://172.28.129.39:8080/shot.jpg" #Lab3 WiFi
#url = "http://172.28.131.227:8080/shot.jpg" #Lab2 WiFi
url = "http://192.168.43.1:8080/shot.jpg" #peter's phone
#url = "http://192.168.43.7:8080/shot.jpg"  #eldoor el tani
#url = "http://192.168.1.100:8080/shot.jpg" #Home WiFi
crop_value = 0.5
#def getleft(image):
#    x = int(image.shape[1]/2)
#    y = int(image.shape[0]-1)
#    
#    for i in range(x,-1,-1):
#        for j in range(y,-1,-1):
#            if image[j][i] == 255 :
#                return i
#
#def getright(image):
#    x = int(image.shape[1]/2)-1
#    y = int(image.shape[0]/2)
#    
#    for i in range(x,image.shape[1]):
#        #print(str(i))
#        for j in range(y,-1,-1):
#            #print(str(j))
#            if image[j][i] == 255 :
#                return i
def detect_edges(img):
    
    #gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(img,(3,3),0)
    canny = cv2.Canny(blur, 30, 120)

    return canny

def regionOfInterest(canny):
    height, width = canny.shape
    #left = getleft(image)
    #right = getright(image)
    mask = np.zeros_like(canny)

    polygons = np.array([
            [(0, height* crop_value),(width,height* crop_value),(width,height),(0,height)]
            ],np.int32)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(canny,mask)
    return masked_image

def detect_line_segments(masked_image):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(masked_image, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments

def average_slope_intercept(img, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width,_ = img.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(img, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(img, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines

def make_points(img, line):
    height, width,_ = img.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def detect_lane(img):
    
    canny = detect_edges(img)
    masked_image = regionOfInterest(canny)
    filt = myFilter(img)
    out = cv2.bitwise_and(masked_image,filt)    
    line_segments = detect_line_segments(out)
    lane_lines = average_slope_intercept(img, line_segments)

    
    return lane_lines

#def action(image,left,right):
#    #print("l=  "+left)
#    #print("r="+right)
#    mid = int(abs(left - right)/2)
#    current = int(image.shape[1]/2)
#    if mid > current - 20 :
#        print("R")
#        #send("R")
#    elif mid < current + 20 :
#        #send("L")
#        print("L")
#    else :
#        print("F")
#        #send("F")
def display_lines(img, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(img, 0.8, line_image, 1, 1)
    return line_image

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width,_ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle


def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
   """
   Using last steering angle to stabilize the steering angle
   This can be improved to use last N angles, etc
   if new angle is too different from current angle, only turn by max_angle_deviation degrees
   """
   if num_of_lane_lines == 2 :
       # if both lane lines detected, then we can deviate more
       max_angle_deviation = max_angle_deviation_two_lines
   else :
       # if only one lane detected, don't deviate too much
       max_angle_deviation = max_angle_deviation_one_lane
   
   angle_deviation = new_steering_angle - curr_steering_angle
   if abs(angle_deviation) > max_angle_deviation:
       stabilized_steering_angle = int(curr_steering_angle
                                       + max_angle_deviation * angle_deviation / abs(angle_deviation))
   else:
       stabilized_steering_angle = new_steering_angle
   logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
   return stabilized_steering_angle


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width,_ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def sendData(char):
    try :
        s = serial.Serial('COM12',9600,timeout =1)
        data = bytearray(char , 'utf-8')
        s.write(data)
    except :
        sendData(char)

def myFilter(image):
    hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    im10=hsv_img[:,:,0]
    for i in range(im10.shape[0]) :
        for j in range(im10.shape[1]):
            if (im10[i][j] > 110):
                im10[i][j] = 0 

    x = copy.deepcopy(im10)
    #cv2.imshow("filter", x)

    return x     

Last_steering_angle = None
steering_angle = None
counter = 0
while True:
    
    #result = firebase.get('/task3/data', None)
    #if result == "1":
        img_resp = requests.get(url)
        img_arr = np.array(bytearray(img_resp.content) , dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        
        #filt = myFilter(img)
        
        lane_lines = detect_lane(img)
        
        lane_lines_image = display_lines(img, lane_lines)
        Last_steering_angle = copy.deepcopy(steering_angle)
        steering_angle = compute_steering_angle(img,lane_lines)
        if Last_steering_angle == None :
            Last_steering_angle = steering_angle
        
        stabilized_steering_angle = stabilize_steering_angle(Last_steering_angle,steering_angle,len(lane_lines))
        
       
        if stabilized_steering_angle == -90 :
            char = "B"
        elif stabilized_steering_angle < 75 or stabilized_steering_angle == 75:
            print("Turn Left")
            char = "L"
            #if stabilized_steering_angle < 60 :
                #char = "S"
                #print("unacceptable angle so stop")
        elif stabilized_steering_angle > 105 or stabilized_steering_angle == 105  :
                print("Turn Right")
                char = "R"
                #if stabilized_steering_angle > 130 :
                #    char = "S"
                #    print("unacceptable angle so stop")
        elif stabilized_steering_angle > 75 and stabilized_steering_angle < 105 :
            print("Move Forward")
            char = "F"
        else :
            print("Stop")
            char = "S"
        #if counter == 5 :
        #    char = "R"
        sendData(char)
        if char == "B" :
            time.sleep(2)
            sendData("R")
            time.sleep(1)
        else :
            time.sleep(0.5)
        sendData("F")
        time.sleep(0.05)
        
        print(steering_angle)
        counter=counter+1
        if counter % 3 == 0 :
            sendData("S")
            time.sleep(1)
            counter = 0
        heading_image=display_heading_line(lane_lines_image,steering_angle,line_color=(0, 0, 255), line_width=10)
        cv2.imshow("Cam", heading_image)
    
        if cv2.waitKey(1)==27:
            break

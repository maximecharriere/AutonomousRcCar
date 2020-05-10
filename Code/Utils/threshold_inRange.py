#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            OpenCV community (last modified by Maxime Charriere)
#   Project:           Open CV
#   File:              threshold_inRange.py
#   Link:              https://github.com/opencv/opencv/blob/master/samples/python/tutorial_code/imgProc/threshold_inRange/threshold_inRange.py
#   Creation date :    18.05.2018
#   Last modif date:   01.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   GUI with sliders to choose the perfect HSV value in order to 
#   proceed to an threshold
#   A still image of a camera feed can be used
## -------------------------------- Description --------------------------------

from __future__ import print_function
import cv2 as cv
import argparse
import picamera
import numpy as np
import time
import myLib

max_value = 255
max_value_H = 360//2

low_H = 0
high_H = max_value_H
low_S = 0
high_S = max_value
low_V = 0
high_V = max_value

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

## [low]
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
## [low]

## [high]
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
## [high]

def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)

def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)

def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)

def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()


## [window]
cv.namedWindow(window_capture_name, cv.WINDOW_NORMAL)
cv.namedWindow(window_detection_name, cv.WINDOW_NORMAL)
## [window]

## [trackbar]
cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
## [trackbar]

## If a still image is set
# cap = cv.VideoCapture("/home/pi/Documents/AutonomousRcCar/Images/ConfigCamera/2020-04-14_14-15-34_cts-100_DRC-high_sat-100_sharp-100_awbr-1.3_awbb-1.6_expMode-auto_expSpeed-30569.jpg") #args.camera
# ret, frame = cap.read()

frameBGR = np.empty((1920, 2592, 3), dtype=np.uint8)
with picamera.PiCamera(resolution=(2592, 1920), framerate=30, sensor_mode=2) as camera: 
    time.sleep(2)  
    myLib.PrintCamInfos(camera)

    while True:
        ## [while]
        
        ## If a camera is set
        camera.capture(frameBGR, 'bgr')
        #if frame is None:
        #    break

        frame_HSV = cv.cvtColor(frameBGR, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
        ## [while]

        ## [show]
        cv.imshow(window_capture_name, frameBGR)
        cv.imshow(window_detection_name, frame_threshold)
        ## [show]

        ## [quit]
        key = cv.waitKey(5)
        if key == ord('q') or key == 27:
            break
        ## [quit]
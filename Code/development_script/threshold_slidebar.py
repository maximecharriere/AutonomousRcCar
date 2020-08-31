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

import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

import cv2 as cv
import argparse
import picamera
import picamera.array
import numpy as np
import time
import my_lib
from img_warper import ImgWarper
from img_rectifier import ImgRectifier
from camera_controller import PicameraController

CONFIG_FNAME = os.path.join(parentdir, 'conf.yaml')
conf = my_lib.load_configuration(CONFIG_FNAME)

max_value = 255
max_value_H = 360//2

low_H = conf["ROAD_FOLLOWING"]["hsv_threshold"]["low"][0]
low_S = conf["ROAD_FOLLOWING"]["hsv_threshold"]["low"][1]
low_V = conf["ROAD_FOLLOWING"]["hsv_threshold"]["low"][2]
high_H = conf["ROAD_FOLLOWING"]["hsv_threshold"]["high"][0]
high_S = conf["ROAD_FOLLOWING"]["hsv_threshold"]["high"][1]
high_V = conf["ROAD_FOLLOWING"]["hsv_threshold"]["high"][2]


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
    # low_H = min(high_H-1, low_H)
    # cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
## [low]

## [high]
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    # high_H = max(high_H, low_H+1)
    # cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
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




camera = PicameraController(
    cam_param_dict = [(arg, value) for (arg, value) in conf['CAMERA']['parameters'].items() if value != None])
imgRectifier = ImgRectifier(
    imgShape = camera.resolution[::-1],
    calParamFile = os.path.join(parentdir, conf["ROAD_FOLLOWING"]["calibration"]["param_file"]))
imgWarper = ImgWarper(
    imgShape = camera.resolution[::-1], 
    corners = conf["ROAD_FOLLOWING"]["perspective_warp"]["points"], 
    realWorldCornersDistance = conf["ROAD_FOLLOWING"]["perspective_warp"]["realworld_line_distance"], 
    margin_pc = conf["ROAD_FOLLOWING"]["perspective_warp"]["warp_margin"], 
    cornersImageResolution = conf["ROAD_FOLLOWING"]["perspective_warp"]["points_resolution"])

while True:
    img = camera.capture_np()
    img = imgRectifier.undistort(img)
    img_warped = imgWarper.warp(img)
    img_HSV = cv.cvtColor(img_warped, cv.COLOR_RGB2HSV)

    img_threshold = my_lib.inRangeHSV(img_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    ## [show]
    cv.imshow(window_capture_name, cv.cvtColor(img_warped, cv.COLOR_RGB2BGR))
    cv.imshow(window_detection_name, img_threshold)
    ## [show]

    ## [quit]
    key = cv.waitKey(1)
    if key == ord('q') or key == 27:
        break
    ## [quit]


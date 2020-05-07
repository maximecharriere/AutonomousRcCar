#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              roadFollowing.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    26.03.2020
#   Last modif date:   04.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   Main file to compute the road detection
## -------------------------------- Description --------------------------------

import time
import picamera
import picamera.array
import numpy as np
import cv2
from datetime import datetime
from CameraCalibration import cameraCalibration
from PerspectiveWarp import perspectiveWarp

## Parameters
low_H = 0
low_S = 157
low_V = 254
high_H = 22
high_S = 255
high_V = 255
SaveFirstFrame = False
ShowPreview = False

with picamera.PiCamera(resolution=(2592, 1944), sensor_mode=2) as camera:
    with picamera.array.PiRGBArray(camera, size=(2592, 1944)) as rawCapture :
        ## Camera configuration
#         camera.exposure_mode = 'auto' 
#         camera.meter_mode = 'average'
#         camera.contrast = 100
#         camera.drc_strength = 'high'
#         camera.saturation = 100
#         camera.sharpness = 100
        # camera.shutter_speed = 62974
        # camera.ISO = 100
        # camera.brightness = 50
        # camera.awb_mode  = 'off'
        #camera.awb_gains = (350/256,400/256)

        ## Let time to the camera for color and exposure calibration 
        time.sleep(2)  

        ## Preview
        if ShowPreview:
            camera.start_preview()

        ## Save the picture with camera parameters in filename
        if SaveFirstFrame:
            camera.capture(f"Images/ConfigCamera/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_cts-{camera.contrast}_DRC-{camera.drc_strength}_sat-{camera.saturation}_sharp-{camera.sharpness}_awbr-{float(camera.awb_gains[0]):.1f}_awbb-{float(camera.awb_gains[1]):.1f}_expMode-{camera.exposure_mode}_expSpeed-{camera.exposure_speed}.jpg")
        
        for frame in camera.capture_continuous(rawCapture , format="bgr", use_video_port=True):
            frameBGR = frame.array
            frameBGR_calibrate = cameraCalibration.undistort(frameBGR, calParamFile="/home/pi/Documents/AutonomousRcCar/Code/CameraCalibration/cameraCalibrationParam_V2.pickle",crop=True)
            frameBGR_warped = perspectiveWarp.perspective_warp(frameBGR_calibrate, [(1120, 760),(1600, 760),(313, 1444),(2435, 1444)], [400, 400, 400, 400])
            frameHSV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HSV)
            frameThreshold = cv2.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))
            frameEdge = cv2.Canny(frameThreshold, 100,400)


            ## Show image
            cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Threshold", cv2.WINDOW_NORMAL)
            cv2.imshow("Original", frameBGR)
            cv2.imshow("Threshold", frameBGR_warped)

            key = cv2.waitKey(1)

            rawCapture.truncate(0)

            if key == ord("q"):
                break
            
        ## Close properly
        camera.stop_preview()
        cv2.destroyAllWindows()
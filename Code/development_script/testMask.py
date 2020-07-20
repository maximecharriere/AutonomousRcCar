#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              app.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    26.03.2020
#   Last modif date:   04.05.2020
# ----------------------------------- Infos -----------------------------------

# -------------------------------- Description --------------------------------
#   Main file to compute the road detection
# -------------------------------- Description --------------------------------

import time
from datetime import datetime
import picamera
import picamera.array
import numpy as np
import cv2
import matplotlib.pyplot as plt
import my_lib
import camera_calibration
import perspective_warp
from pwmcontroller import SteeringController, SpeedController
from scipy import stats

# Parameters

# Pin declaration with BCM format
PIN_SPEED = 18
PIN_STEERING = 19

# (2592, 1952) and not (2592, 1944) because high must be a multiple of 16
camResolution = (640, 480)
min_line_area = 0.1  # in % of img area
low_H = 175
low_S = 100 #70
low_V = 50
high_H = 30
high_S = 255
high_V = 255
perspectiveWarpPoints = [(173, 1952), (2560, 1952), (870, 920), (1835, 920)]
perspectiveWarpPointsResolution = (2592, 1952)
maxSD = 0.01
SaveFirstFrame = False
ShowCamPreview = False
ShowPlot = True

def start():
    with picamera.PiCamera(resolution=camResolution, sensor_mode=2) as camera:
        with picamera.array.PiRGBArray(camera, size=camResolution) as rawCapture:
            (bg, rg) = camera.awb_gains
            camera.awb_mode = 'off'
            camera.awb_gains = (1, 211/128)
            camera.contrast=50
            camera.saturation=100
            camera.sharpness=0
            # Let time to the camera for color and exposure calibration
            time.sleep(1)

            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                frameBGR = frame.array
                frameBGR_calibrate = camera_calibration.undistort(
                    frameBGR, calParamFile="/home/pi/Documents/AutonomousRcCar/autonomouscar/resources/cameraCalibrationParam_V2.pickle", crop=True)
                frameBGR_warped = perspective_warp.warp(frameBGR_calibrate, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameHSV = cv2.cvtColor(frameBGR_calibrate, cv2.COLOR_BGR2HSV)

                # Reset analised frame
                rawCapture.truncate(0)


                cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
                cv2.imshow("Original", frameBGR)

                key = cv2.waitKey(1)
                if key == ord("q"):
                    break

            # Close properly
            cv2.destroyAllWindows()

if __name__ == '__main__':
    start()

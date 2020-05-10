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
from Utils import myLib
import picamera
import picamera.array
import numpy as np
import cv2
from datetime import datetime
import matplotlib.pyplot as plt
from CameraCalibration import cameraCalibration
from PerspectiveWarp import perspectiveWarp

## Parameters
min_line_area = 500 #in pixel
low_H = 0
low_S = 0
low_V = 1
high_H = 180
high_S = 255
high_V = 90
SaveFirstFrame = False
ShowPreview = False

def secross(radius=1):
    A = np.abs(np.arange(-radius,radius+1))
    dists = A[:,None] + A
    return ((dists - radius)<=0)

with picamera.PiCamera(resolution=(2592, 1944), sensor_mode=2) as camera:
    with picamera.array.PiRGBArray(camera, size=(2592, 1944)) as rawCapture :
        ## Let time to the camera for color and exposure calibration 
        time.sleep(0.5)  

        ## Preview
        if ShowPreview:
            camera.start_preview()

        ## Save the picture with camera parameters in filename
        if SaveFirstFrame:
            camera.capture(f"Images/ConfigCamera/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_cts-{camera.contrast}_DRC-{camera.drc_strength}_sat-{camera.saturation}_sharp-{camera.sharpness}_awbr-{float(camera.awb_gains[0]):.1f}_awbb-{float(camera.awb_gains[1]):.1f}_expMode-{camera.exposure_mode}_expSpeed-{camera.exposure_speed}.jpg")
        
        for frame in camera.capture_continuous(rawCapture , format="bgr", use_video_port=True):
            frameBGR = frame.array
            frameBGR_calibrate = cameraCalibration.undistort(frameBGR, calParamFile="/home/pi/Documents/AutonomousRcCar/Code/CameraCalibration/cameraCalibrationParam_V2.pickle",crop=True)
            frameBGR_warped = perspectiveWarp.perspective_warp(frameBGR_calibrate, [(60, 1900),(2560, 1900),(1017, 680),(1621, 680)], [500, 0, 500, 0])
            frameHSV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HSV)

            ## Canny
            frameThreshold = cv2.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))
            # frameEdge = cv2.Canny(frameThreshold, 100,400)

            # ## Sobel
            # v_channel = frameHSV[:,:,2]
            # sobelx = cv2.Sobel(v_channel, cv2.CV_16S, 1, 0) # Take the derivative in x
            # abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
            # scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
            
            ## Connected components
            frameThreshold = cv2.erode(frameThreshold,kernel=np.ones((3,3))) #to minimise the number of components and speed processing time (à mesurer)
            retval, labels_img, stats, centroids = cv2.connectedComponentsWithStats(frameThreshold, ltype=cv2.CV_16U) #https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html
            line_label = np.where(stats[1:,cv2.CC_STAT_AREA] >= min_line_area)[0]+1 #1 is to exclude label 0 who is the background 
            
            #generate image with label
            labelizedThreshold = np.zeros_like(labels_img)
            step = int(255/line_label.size)
            for i in range(line_label.size):
                labelizedThreshold[np.where(labels_img == line_label[i])] = step*(i+1)
            ## Polyfit
            # [nonzeroy ,nonzerox ] = frameThreshold.nonzero()
            # coef = np.polyfit(nonzerox, nonzeroy, 2)
            # draw_x = np.linspace(0, frameThreshold.shape[1]-1, frameThreshold.shape[1], dtype=int)
            # draw_y = np.polyval(coef, draw_x)
            # draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32)
            # coloredImg = np.dstack((frameThreshold, frameThreshold, frameThreshold))
            # cv2.polylines(coloredImg, [draw_points], False, (255,0,0), 20)


            ## Show image
            plt.imshow(labelizedThreshold, cmap="gray")
            plt.title("Pixels grouping")
            plt.show()

            # fig, axs = plt.subplots(2, 3)
            # axs[0,0].imshow(h_channelV, cmap="gray")
            # axs[0,0].set_title("HSV H channel")
            # axs[0,1].imshow(s_channelV, cmap="gray")
            # axs[0,1].set_title("HSV S channel")
            # axs[0,2].imshow(v_channelV, cmap="gray")
            # axs[0,2].set_title("HSV V channel")
            # axs[1,0].imshow(h_channelL, cmap="gray")
            # axs[1,0].set_title("HLS H channel")
            # axs[1,1].imshow(s_channelL, cmap="gray")
            # axs[1,1].set_title("HLS S channel")
            # axs[1,2].imshow(l_channelL, cmap="gray")
            # axs[1,2].set_title("HLS L channel")
            # plt.show()
            
            # cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
            # cv2.namedWindow("Warped", cv2.WINDOW_NORMAL)
            # cv2.namedWindow("Threshold", cv2.WINDOW_NORMAL)
            # cv2.imshow("Original", frameBGR_calibrate)
            # cv2.imshow("Warped", frameBGR_warped)
            # cv2.imshow("Threshold", labelizedThreshold)

            key = cv2.waitKey(1)

            rawCapture.truncate(0)

            if key == ord("q"):
                break
            
        ## Close properly
        camera.stop_preview()
        cv2.destroyAllWindows()
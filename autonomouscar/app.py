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
import math
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
camResolution = (640, 480)#(2592, 1952)
min_line_area = 0.1  # in % of img area
lineColor_right = (255,0,0) #bgr
lineColor_left  = (255,255,0)
lineColor_rejected = (0,0,255)
textColor = (0,255,255)
low_H = 2#175
low_S = 60#90
low_V = 80 #50
high_H = 40
high_S = 215
high_V = 255
perspectiveWarpPoints = [(896, 920), (1819, 925), (2461, 1800), (262, 1803)]#[(173, 1952), (2560, 1952), (870, 920), (1835, 920)]
perspectiveWarpPointsResolution = (2592, 1952)
lineSpacing = 574
maxSD = 0.008
SaveFirstFrame = False
ShowCamPreview = False
ShowPlot = True
slop_margin = 0.5
slop_history = {
    "lastValue": 0.0, 
    "lastUpdate" : 1000
}



## Objects
SpeedCtrl = SpeedController(PIN_SPEED,5.5,9.5)
SteeringCtrl = SteeringController(PIN_STEERING, 6.5, 9.5)

def start():
    with picamera.PiCamera(resolution=camResolution, sensor_mode=2) as camera:
        with picamera.array.PiRGBArray(camera, size=camResolution) as rawCapture:
            # (bg, rg) = camera.awb_gains
            # camera.awb_mode = 'off'
            # camera.awb_gains = (1, 211/128)
            # camera.contrast=50
            # camera.saturation=100
            # camera.sharpness=0
            # Let time to the camera for color and exposure calibration
            time.sleep(1)

            # Preview
            if ShowCamPreview:
                camera.start_preview()

            # Save the picture with camera parameters in filename
            if SaveFirstFrame:
                camera.capture(
                    f"Images/ConfigCamera/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_cts-{camera.contrast}_DRC-{camera.drc_strength}_sat-{camera.saturation}_sharp-{camera.sharpness}_awbr-{float(camera.awb_gains[0]):.1f}_awbb-{float(camera.awb_gains[1]):.1f}_expMode-{camera.exposure_mode}_expSpeed-{camera.exposure_speed}.jpg")

            slop_clamped  = 0
            centerDiff_tan = 0
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                frameBGR = frame.array
                frameBGR_calibrate = camera_calibration.undistort(
                    frameBGR, calParamFile="/home/pi/Documents/AutonomousRcCar/autonomouscar/resources/cameraCalibrationParam_V2.pickle", crop=True)
                frameBGR_warped = perspective_warp.warp(frameBGR_calibrate, perspectiveWarpPoints, (50,50), [80, 0, 80, 10], perspectiveWarpPointsResolution)
                frameBGR_warped2 = perspective_warp.warp(frameBGR, perspectiveWarpPoints, (50,50), [80, 0, 80, 10], perspectiveWarpPointsResolution)
                frameHSV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HSV)

                # Threshold
                frameThreshold = my_lib.inRangeHSV(frameHSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
                # frameThreshold = cv2.morphologyEx(frameThreshold, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15)))
                # to minimise the number of components and speed processing time (à mesurer)
                # frameThreshold = cv2.erode(frameThreshold, kernel=np.ones((3, 3)))
                # frameEdge = cv2.Canny(frameThreshold, 100,400)

                # # Sobel
                # s_channel = cv2.GaussianBlur(frameHSV[:,:,1],(3,3),0)
                # v_channel = cv2.GaussianBlur(frameHSV[:,:,2],(3,3),0)

                # sDerivativeX = cv2.Scharr(s_channel, cv2.CV_16S, 1, 0)  # Take the s channel derivative in x, CV_16S to have neg value
                # sDerivativeY = cv2.Scharr(s_channel, cv2.CV_16S, 0, 1)  # Take the s channel derivative in y
                # sDerivativeX_abs = np.absolute(sDerivativeX)
                # sDerivativeY_abs = np.absolute(sDerivativeY)
                # sDerivative = sDerivativeX_abs + sDerivativeY_abs
                # sDerivative_scaled = np.uint8(sDerivative/np.max(sDerivative)*255)

                # vDerivativeX = cv2.Scharr(v_channel, cv2.CV_16S, 1, 0)  # Take the s channel derivative in x, CV_16S to have neg value
                # vDerivativeY = cv2.Scharr(v_channel, cv2.CV_16S, 0, 1)  # Take the s channel derivative in y
                # vDerivativeX_abs = np.absolute(vDerivativeX)
                # vDerivativeY_abs = np.absolute(vDerivativeY)
                # vDerivative = vDerivativeX_abs + vDerivativeY_abs
                # vDerivative_scaled = np.uint8(vDerivative/np.max(vDerivative)*255)

                # # sDerivative = np.sqrt(sDerivativeX_abs**2+sDerivativeY_abs**2)
                # sDerivative_warped = perspective_warp.warp(sDerivative_scaled, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)

                # Connected components
                _, labels_img, blobStats, _ = cv2.connectedComponentsWithStats(
                    frameThreshold, ltype=cv2.CV_16U)  # https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html
                # the "1" is to exclude label 0 who is the background
                line_label = np.where(
                    blobStats[1:, cv2.CC_STAT_AREA] >= min_line_area*frameThreshold.size/100)[0]+1

                if (line_label.size == 0):
                    slop_history["lastUpdate"]+=1
                    print("No line found")
                else:
                    allCoef = []
                    stdDeviation = []
                    # Polyfit
                    for i in range(line_label.size):
                        y, x = np.where(labels_img == line_label[i])
                        p, V, = np.polyfit(y, x, 1, cov = True) # inversion of x and y because lines are mostly vertical
                        allCoef.append(p)
                        stdDeviation.append(np.sqrt(V[0,0]))

                    # # Linregress
                    # for i in range(line_label.size):
                    #     y, x = np.where(labels_img == line_label[i])
                    #     slope, intercept, r_value, p_value, std_err = stats.linregress(y, x) # inversion of x and y because lines are mostly vertical
                    #     coef.append((slope, intercept))
                    #     stdDeviation.append(r_value**2)

                    # Conversion into np array
                    allCoef = np.array(allCoef)
                    stdDeviation = np.array(stdDeviation)
                    
                    # Selection of correct lines
                    wantedLines_mask = (stdDeviation<maxSD)
                    if(slop_history["lastUpdate"]<5):
                        # print(f"allCoef: {allCoef[:,0]}  -->   slop_history: {slop_history['lastValue']}")
                        wantedLines_mask &= (allCoef[:,0]<slop_history["lastValue"]+slop_margin) & (allCoef[:,0]>slop_history["lastValue"]-slop_margin)
                    
                    # Classify left/right lines
                    bottom = frameThreshold.shape[0]
                    lines_bottom_intercept = np.zeros_like(stdDeviation)
                    lines_bottom_intercept = allCoef[:, 0]*bottom + allCoef[:,1]
                    leftLines_mask = wantedLines_mask & (lines_bottom_intercept <= frameThreshold.shape[1]/2)
                    rightLines_mask = wantedLines_mask & (lines_bottom_intercept > frameThreshold.shape[1]/2)

                    if (np.where(wantedLines_mask)[0].size == 0):
                        slop_history["lastUpdate"]+=1
                        print(f"No correct line found ({slop_history['lastUpdate']})")
                    else:
                        ## Compute the mean slop of lines
                        slop = np.mean(allCoef[wantedLines_mask,0])
                        slop_clamped = my_lib.clamp(slop, -1, 1)
                        # Reset history
                        slop_history["lastUpdate"] = 0
                        slop_history["lastValue"] = slop

                        ## Compute the decentration of the car
                        leftLine_bottom_intercept = lines_bottom_intercept[leftLines_mask].mean()
                        rightLine_bottom_intercept = lines_bottom_intercept[rightLines_mask].mean()

                        if  my_lib.isaN(leftLine_bottom_intercept) and my_lib.isaN(rightLine_bottom_intercept): #both line found
                            roadCenter = np.mean((leftLine_bottom_intercept, rightLine_bottom_intercept))
                        elif my_lib.isaN(leftLine_bottom_intercept): #left line found
                            roadCenter = leftLine_bottom_intercept + lineSpacing/2
                        elif my_lib.isaN(rightLine_bottom_intercept): #right line found
                            roadCenter = rightLine_bottom_intercept - lineSpacing/2
                        else: #no line found
                            raise Exception("No line found. It should not be possible at this point in the code for any line to be found")
                        centerDiff = roadCenter - frameThreshold.shape[1]/2
                        centerDiff_norm_clamped = my_lib.clamp(centerDiff/(lineSpacing/2),-1,1) #Normalize centerDiff that can be between +-lineSpacing/2 to become between +-1
                        centerDiff_tan = np.tan(centerDiff_norm_clamped*np.pi/4)


                    # Change steering
                    car_steering_norm = my_lib.mix(slop_clamped*-1, centerDiff_tan, np.abs(slop_clamped)) #-1 because the slop is reversed
                    car_steering = my_lib.map(car_steering_norm, -1, 1, 0, 100)
                    SteeringCtrl.Angle(car_steering)
                    print(f"slop_clamped: {slop_clamped}   centerDiff_tan: {centerDiff_tan}   car_steering: {car_steering}")
     
        
                # Reset analised frame
                rawCapture.truncate(0)

                # Show result with plots
                if ShowPlot:
                    # Generate image with different color for each label
                    labelizedThreshold = np.zeros_like(frameThreshold)
                    if (line_label.size > 0):
                        colorStep = int(255/line_label.size)
                        for i in range(line_label.size):
                            labelizedThreshold[np.where(
                                labels_img == line_label[i])] = colorStep*(i+1)

                    # Draw polyfit
                    coloredImg = np.dstack((labelizedThreshold, labelizedThreshold, labelizedThreshold))
                    draw_y = np.linspace(0, frameThreshold.shape[0]-1, frameThreshold.shape[0], dtype=int)
                    for i in range(line_label.size): 
                        # Compute points
                        draw_x = np.polyval(allCoef[i,:], draw_y)
                        draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32)
                        # Draw line
                        if rightLines_mask[i]:
                            lineColor = lineColor_right
                        elif leftLines_mask[i]:
                            lineColor = lineColor_left
                        else:
                            lineColor = lineColor_rejected
                        cv2.polylines(coloredImg, [draw_points], False, lineColor, 5)
                        # Draw text
                        textOrg = (blobStats[line_label[i],cv2.CC_STAT_LEFT] + int(blobStats[line_label[i],cv2.CC_STAT_WIDTH]/2), blobStats[line_label[i],cv2.CC_STAT_TOP] + int(blobStats[line_label[i],cv2.CC_STAT_HEIGHT]/2))
                        coloredImg = cv2.putText(coloredImg,f"SD = {stdDeviation[i]:.4f}",textOrg, cv2.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)

                    # Show
                    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Calibrate", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Warped wo calibration", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Warped w calibration", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Polyfit", cv2.WINDOW_NORMAL)
                    cv2.imshow("Original", frameBGR)
                    cv2.imshow("Calibrate", frameBGR_calibrate)
                    cv2.imshow("Warped wo calibration", frameBGR_warped2)
                    cv2.imshow("Warped w calibration", frameBGR_warped)
                    cv2.imshow("Polyfit", coloredImg)

                    # Quit
                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        break

            # Close properly
            camera.stop_preview()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    start()

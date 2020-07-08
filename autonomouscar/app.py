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
import asyncio
from evdev import InputDevice, categorize, ecodes, util
import numpy as np
import cv2
import matplotlib.pyplot as plt
import my_lib
import camera_calibration
import perspective_warp
from car import Car
from scipy import stats
import yaml


class AutonomousCarApp():
    def __init__(self, conf_fname):
        # Load configuration file
        self.conf = my_lib.load_configuration(conf_fname)

        #Objects
        self.Car = Car(self.conf)
        self.Controller = InputDevice(self.conf["CONTROLLER"]["event_filename"])
        self.ImgRectifier = ImgRectifier(conf["IMAGE_PROCESSING"]["calibration"]["param_file"], self.Car.Camera.resolution)

def start():
    # Start a fullscreen preview
    if conf["DISPLAY"]["show_cam_preview"]:
        camera.start_preview()
    car_running = False
    slop_history = {
        "lastValue": 0.0, 
        "lastUpdate" : conf["IMAGE_PROCESSING"]["line_filtering"]["history_size"]+1
    }
    while True:
        self.Car.Camera.capture_np()

            slop_clamped  = 0
            centerDiff_tan = 0

                frameRGB_calibrate = camera_calibration.undistort(
                    img = frameRGB, 
                    calParamFile = conf["IMAGE_PROCESSING"]["calibration"]["param_file"], 
                    crop = True)
                frameRGB_calibrate_crop = camera_calibration.undistort(img=frameRGB, calParamFile=conf["IMAGE_PROCESSING"]["calibration"]["param_file"], crop=False)
                frameRGB_warped = perspective_warp.warp(
                    img = frameRGB_calibrate, 
                    imgPoints = conf["IMAGE_PROCESSING"]["perspective_warp"]["points"], 
                    realWorldPointsDistance = conf["IMAGE_PROCESSING"]["perspective_warp"]["realworld_line_distance"], 
                    margin_pc = conf["IMAGE_PROCESSING"]["perspective_warp"]["warp_margin"], 
                    refImageResolution = conf["IMAGE_PROCESSING"]["perspective_warp"]["points_resolution"])
                frameHSV = cv2.cvtColor(frameRGB_warped, cv2.COLOR_RGB2HSV)

                # Threshold
                frameThreshold = my_lib.inRangeHSV(
                    src = frameHSV, 
                    lowerb = conf["IMAGE_PROCESSING"]["hsv_threshold"]["low"], 
                    upperb = conf["IMAGE_PROCESSING"]["hsv_threshold"]["high"])
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
                    blobStats[1:, cv2.CC_STAT_AREA] >= conf["IMAGE_PROCESSING"]["line_filtering"]["min_area"]*frameThreshold.size/100)[0]+1

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
                    wantedLines_mask = (stdDeviation < conf["IMAGE_PROCESSING"]["line_filtering"]["max_SD"])
                    if(slop_history["lastUpdate"] < conf["IMAGE_PROCESSING"]["line_filtering"]["history_size"]):
                        wantedLines_mask &= (allCoef[:,0]<slop_history["lastValue"]+conf["IMAGE_PROCESSING"]["line_filtering"]["slop_margin"]) & (allCoef[:,0]>slop_history["lastValue"]-conf["IMAGE_PROCESSING"]["line_filtering"]["slop_margin"])
                    
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
                            roadCenter = leftLine_bottom_intercept + conf["IMAGE_PROCESSING"]["line_spacing"]/2
                        elif my_lib.isaN(rightLine_bottom_intercept): #right line found
                            roadCenter = rightLine_bottom_intercept - conf["IMAGE_PROCESSING"]["line_spacing"]/2
                        else: #no line found
                            raise Exception("No line found. It should not be possible at this point in the code for any line to be found")
                        centerDiff = frameThreshold.shape[1]/2 - roadCenter
                        centerDiff_norm_clamped = my_lib.clamp(centerDiff/(conf["IMAGE_PROCESSING"]["line_spacing"]/2),-1,1) #Normalize centerDiff that can be between +-lineSpacing/2 to become between +-1
                        centerDiff_tan = np.tan(centerDiff_norm_clamped*np.pi/4)


                    ## Change steering
                    car_steering_norm = my_lib.mix(slop_clamped, centerDiff_tan, np.abs(slop_clamped))
                    SteeringCtrl.Angle(car_steering_norm)
                    # print(f"slop_clamped: {slop_clamped}   centerDiff_tan: {centerDiff_tan}   car_steering: {car_steering}")

                ## Change speed
                # Controller input
                try:
                    for event in Controller.read():
                        if event.type == ecodes.EV_KEY:
                            if event.value == True:
                                if event.code == ecodes.ecodes[conf["CONTROLLER"]["btn_stop"]]: 
                                    if car_running: 
                                        SpeedCtrl.Stop()
                                        car_running = False
                                        print("STOP")
                                elif  event.code == ecodes.ecodes[conf["CONTROLLER"]["btn_start"]]:
                                    SpeedCtrl.Speed(conf["CAR"]["speed"])
                                    car_running = True
                                    print("RUN")
                except BlockingIOError:
                    pass

                # Stop if no lines found
                if (slop_history["lastUpdate"] >= conf["IMAGE_PROCESSING"]["line_filtering"]["history_size"]):
                    if car_running:
                        SpeedCtrl.Stop()
                        car_running = False
                        print("STOP")

                # Reset analised frame
                rawCapture.truncate(0)

                # Show result with plots
                if conf["DISPLAY"]["show_plots"]:
                    # Generate image with different color for each connected component
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
                            lineColor = conf["DISPLAY"]["linecolor_right"]
                        elif leftLines_mask[i]:
                            lineColor = conf["DISPLAY"]["lineColor_left"]
                        else:
                            lineColor = conf["DISPLAY"]["lineColor_rejected"]
                        cv2.polylines(coloredImg, [draw_points], False, lineColor, 5)
                        # Draw text
                        textOrg = (blobStats[line_label[i],cv2.CC_STAT_LEFT] + int(blobStats[line_label[i],cv2.CC_STAT_WIDTH]/2), blobStats[line_label[i],cv2.CC_STAT_TOP] + int(blobStats[line_label[i],cv2.CC_STAT_HEIGHT]/2))
                        coloredImg = cv2.putText(coloredImg,f"SD = {stdDeviation[i]:.4f}",textOrg, cv2.FONT_HERSHEY_SIMPLEX, 1, conf["DISPLAY"]["textColor"], 2)

                    # Show
                    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Calibrate", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Calibrate uncrop", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Warped w calibration", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Polyfit", cv2.WINDOW_NORMAL)
                    cv2.imshow("Original", cv2.cvtColor(frameRGB, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Calibrate", cv2.cvtColor(frameRGB_calibrate, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Calibrate uncrop", cv2.cvtColor(frameRGB_calibrate_crop, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Warped w calibration", cv2.cvtColor(frameRGB_warped, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Polyfit", cv2.cvtColor(coloredImg, cv2.COLOR_RGB2BGR))

                    # Quit
                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        break

            # Close properly
            camera.stop_preview()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    start()



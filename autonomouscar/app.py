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

# Parameters

# Pin declaration with BCM format
PIN_SPEED = 18
PIN_STEERING = 19

# (2592, 1952) and not (2592, 1944) because high must be a multiple of 16
camResolution = (640, 480)
min_line_area = 0.1  # in % of img area
low_H = 0
low_S = 0
low_V = 0
high_H = 180
high_S = 255
high_V = 90
perspectiveWarpPoints = [(173, 1952), (2560, 1952), (870, 920), (1835, 920)]
perspectiveWarpPointsResolution = (2592, 1952)
SaveFirstFrame = False
ShowCamPreview = False
ShowPlot = True

## Objects
SpeedCtrl = SpeedController(PIN_SPEED,5.5,9.5)
SteeringCtrl = SteeringController(PIN_STEERING, 4, 10)

def start():
    with picamera.PiCamera(resolution=camResolution, sensor_mode=2) as camera:
        with picamera.array.PiRGBArray(camera, size=camResolution) as rawCapture:
            # Let time to the camera for color and exposure calibration
            time.sleep(1)

            # Preview
            if ShowCamPreview:
                camera.start_preview()

            # Save the picture with camera parameters in filename
            if SaveFirstFrame:
                camera.capture(
                    f"Images/ConfigCamera/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_cts-{camera.contrast}_DRC-{camera.drc_strength}_sat-{camera.saturation}_sharp-{camera.sharpness}_awbr-{float(camera.awb_gains[0]):.1f}_awbb-{float(camera.awb_gains[1]):.1f}_expMode-{camera.exposure_mode}_expSpeed-{camera.exposure_speed}.jpg")

            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                frameBGR = frame.array
                frameBGR_calibrate = camera_calibration.undistort(
                    frameBGR, calParamFile="/home/pi/Documents/AutonomousRcCar/Code/CameraCalibration/cameraCalibrationParam_V2.pickle", crop=True)
                frameBGR_warped = perspective_warp.warp(frameBGR_calibrate, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameHSV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HSV)

                # Threshold
                frameThreshold = cv2.inRange(
                    frameHSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
                # to minimise the number of components and speed processing time (à mesurer)
                frameThreshold = cv2.erode(frameThreshold, kernel=np.ones((3, 3)))
                # frameEdge = cv2.Canny(frameThreshold, 100,400)

                # ## Sobel
                # v_channel = frameHSV[:,:,2]
                # sobelx = cv2.Sobel(v_channel, cv2.CV_16S, 1, 0) # Take the derivative in x
                # abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
                # scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))

                # Connected components
                retval, labels_img, stats, centroids = cv2.connectedComponentsWithStats(
                    frameThreshold, ltype=cv2.CV_16U)  # https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html
                # the "1" is to exclude label 0 who is the background
                line_label = np.where(
                    stats[1:, cv2.CC_STAT_AREA] >= min_line_area*frameThreshold.size/100)[0]+1

                if (line_label.size > 0):
                    # Polyfit
                    coef = []
                    for i in range(line_label.size):
                        y, x = np.where(labels_img == line_label[i])
                        # inversion of x and y because lines are mostly vertical
                        coef.append(np.polyfit(y, x, 1))

                    coef = np.array(coef)
                    slop = np.mean(coef[:, 0])

                else:
                    print("No line found")

                # Change steering
                angle = my_lib.map(slop, 1, -1, 35, 65)
                print(f"Slop: {slop}  Angle: {angle}")
                SteeringCtrl.Angle(angle)

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
                    coloredImg = np.dstack(
                        (labelizedThreshold, labelizedThreshold, labelizedThreshold))
                    for i in range(line_label.size):
                        draw_y = np.linspace(
                            0, frameThreshold.shape[0]-1, frameThreshold.shape[0], dtype=int)
                        draw_x = np.polyval(coef[i], draw_y)
                        draw_points = (np.asarray(
                            [draw_x, draw_y]).T).astype(np.int32)
                        cv2.polylines(
                            coloredImg, [draw_points], False, (255, 0, 0), 5)

                    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Calibrate", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Warped", cv2.WINDOW_NORMAL)
                    cv2.namedWindow("Polyfit", cv2.WINDOW_NORMAL)
                    cv2.imshow("Original", frameBGR)
                    cv2.imshow("Calibrate", frameBGR_calibrate)
                    cv2.imshow("Warped", frameBGR_warped)
                    cv2.imshow("Polyfit", coloredImg)

                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        break

            # Close properly
            camera.stop_preview()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    start()

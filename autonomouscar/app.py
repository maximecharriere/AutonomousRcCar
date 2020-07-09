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
from camera_calibration import ImgRectifier
from perspective_warp import ImgWarper
from road_follower import RoadFollower
from car import Car
from scipy import stats


class AutonomousCarApp():
    def __init__(self, conf_fname):
        # Load configuration file
        self.conf = my_lib.load_configuration(conf_fname)
        self.speed_infos = {
            'speed_limit' : self.conf["CAR"]["default_speed_limit"],
            'car_stopped' : False
        }
        #Objects
        self.car = Car(self.conf)
        self.controller = InputDevice(self.conf["CONTROLLER"]["event_filename"])
        self.imgRectifier = ImgRectifier(
            imgShape = self.car.camera.resolution,
            calParamFile = self.conf["IMAGE_PROCESSING"]["calibration"]["param_file"])
        self.roadFollower = RoadFollower(
            imgShape = self.car.camera.resolution, 
            conf = self.conf)
        

    def start(self):
        # Start a fullscreen raw preview
        if conf["DISPLAY"]["show_cam_preview"]:
            self.car.camera.start_preview()
        
        while True:
            img = self.car.camera.capture_np()
            img = self.imgRectifier.undistort(img)
            steering_value = self.roadFollower.getSteering(img)
            self.car.steeringCtrl.angle(steering_value)

            ## Change speed
            # Controller input
            try:
                for event in self.controller.read():
                    if event.type == ecodes.EV_KEY:
                        if event.value == True:
                            if event.code == ecodes.ecodes[conf["CONTROLLER"]["btn_stop"]]: 
                                if car_running: 
                                    car_running = False
                                    print("STOP")
                            elif  event.code == ecodes.ecodes[conf["CONTROLLER"]["btn_start"]]:
                                car_running = True
                                print("RUN")
            except BlockingIOError:
                pass
            
            

                # # Show result with plots
                # if conf["DISPLAY"]["show_plots"]:
                #     # Generate image with different color for each connected component
                #     labelizedThreshold = np.zeros_like(frameThreshold)
                #     if (line_label.size > 0):
                #         colorStep = int(255/line_label.size)
                #         for i in range(line_label.size):
                #             labelizedThreshold[np.where(
                #                 labels_img == line_label[i])] = colorStep*(i+1)

                #     # Draw polyfit
                #     coloredImg = np.dstack((labelizedThreshold, labelizedThreshold, labelizedThreshold))
                #     draw_y = np.linspace(0, frameThreshold.shape[0]-1, frameThreshold.shape[0], dtype=int)
                #     for i in range(line_label.size): 
                #         # Compute points
                #         draw_x = np.polyval(allCoef[i,:], draw_y)
                #         draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32)
                #         # Draw line
                #         if rightLines_mask[i]:
                #             lineColor = conf["DISPLAY"]["linecolor_right"]
                #         elif leftLines_mask[i]:
                #             lineColor = conf["DISPLAY"]["lineColor_left"]
                #         else:
                #             lineColor = conf["DISPLAY"]["lineColor_rejected"]
                #         cv2.polylines(coloredImg, [draw_points], False, lineColor, 5)
                #         # Draw text
                #         textOrg = (blobStats[line_label[i],cv2.CC_STAT_LEFT] + int(blobStats[line_label[i],cv2.CC_STAT_WIDTH]/2), blobStats[line_label[i],cv2.CC_STAT_TOP] + int(blobStats[line_label[i],cv2.CC_STAT_HEIGHT]/2))
                #         coloredImg = cv2.putText(coloredImg,f"SD = {stdDeviation[i]:.4f}",textOrg, cv2.FONT_HERSHEY_SIMPLEX, 1, conf["DISPLAY"]["textColor"], 2)

                #     # Show
                #     cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
                #     cv2.namedWindow("Calibrate", cv2.WINDOW_NORMAL)
                #     cv2.namedWindow("Calibrate uncrop", cv2.WINDOW_NORMAL)
                #     cv2.namedWindow("Warped w calibration", cv2.WINDOW_NORMAL)
                #     cv2.namedWindow("Polyfit", cv2.WINDOW_NORMAL)
                #     cv2.imshow("Original", cv2.cvtColor(frameRGB, cv2.COLOR_RGB2BGR))
                #     cv2.imshow("Calibrate", cv2.cvtColor(frameRGB_calibrate, cv2.COLOR_RGB2BGR))
                #     cv2.imshow("Calibrate uncrop", cv2.cvtColor(frameRGB_calibrate_crop, cv2.COLOR_RGB2BGR))
                #     cv2.imshow("Warped w calibration", cv2.cvtColor(frameRGB_warped, cv2.COLOR_RGB2BGR))
                #     cv2.imshow("Polyfit", cv2.cvtColor(coloredImg, cv2.COLOR_RGB2BGR))

                #     # Quit
                #     key = cv2.waitKey(1)
                #     if key == ord("q"):
                #         break

        # Close properly
        self.car.camera.stop_preview()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    start()



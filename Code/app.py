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
from road_follower import RoadFollower
from objects_detector import ObjectsDetector
from obstacle_detector import ObstacleDetector
from car import Car
from scipy import stats
import threading
import copy
import os

CONFIG_FNAME = '/home/pi/Documents/AutonomousRcCar/autonomouscar/conf.yaml'

class AutonomousCarApp():
    def __init__(self, conf_fname):
        # Load configuration file
        self.conf = my_lib.load_configuration(conf_fname)
        self.car_state = {
            'stop_flags': {
                'no_road'     : False,
                'stop_sign'   : False,
                'red_light'   : False,
                'obstacle'    : False,
                'manual_stop' : False
            },
            'speed_limit'     : self.conf["CAR"]["default_speed_limit"]
        }
        self.car_state_history = copy.deepcopy(self.car_state)
        self.threads_fps = {
            'Main'              : 0,
            'PicameraController': 0,
            'RoadFollower'      : 0,
            'ObjectsDetector'   : 0,
            'ObstacleDetector'  : 0
        }
        self.min_execution_time = 1/30
        #Objects
        self.car = Car(
            conf = self.conf, 
            current_threads_fps = self.threads_fps)
        self.roadFollower = RoadFollower(
            conf = self.conf,
            camera = self.car.camera, 
            steeringCtrl = self.car.steeringCtrl, 
            car_state = self.car_state,
            current_threads_fps = self.threads_fps)
        self.objectDetector = ObjectsDetector(
            conf = self.conf, 
            camera  = self.car.camera, 
            car_state = self.car_state,
            max_fps = self.conf['OBJECT_DETECTION']['max_fps'],
            current_threads_fps = self.threads_fps)
        self.obstacleDetector = ObstacleDetector(
            min_distance = self.conf['PROXIMITY']['min_distance'], 
            distance_sensor = self.car.ultrasonicSensor, 
            car_state = self.car_state,
            max_fps = self.conf['PROXIMITY']['max_fps'],
            current_threads_fps = self.threads_fps)

        try:
            self.controller = InputDevice(self.conf["CONTROLLER"]["event_filename"])
            self.car_state['stop_flags']['manual_stop'] = True
        except FileNotFoundError:
            self.controller = None
            print("A wrong gamepad event filename is provided or the gamepad is not connected !")
        
    def start(self):
        if self.conf["DISPLAY"]["show_plots"]:
            cv2.namedWindow("ObjectsDetector", cv2.WINDOW_NORMAL)
            cv2.namedWindow("RoadFollower", cv2.WINDOW_NORMAL)
                
        with self.car: #start motor, steering commande and camera
            # with self.roadFollower: #start the road following algorithm
                with self.objectDetector: #start the sign detector algorithm
                    # with self.obstacleDetector: #start the obstacle detector algorithm
                        print('----------------  ACTIVE THREAD  ----------------')
                        for thread in threading.enumerate():
                            print(thread)

                        # start_time = time.time()
                        while True:
                            # Controller input
                            if self.controller:
                                try:
                                    for event in self.controller.read():
                                        if event.type == ecodes.EV_KEY:
                                            if event.value == True:
                                                if event.code == ecodes.ecodes[self.conf["CONTROLLER"]["btn_stop"]]: 
                                                    self.car_state['stop_flags']['manual_stop'] = True
                                                elif  event.code == ecodes.ecodes[self.conf["CONTROLLER"]["btn_start"]]:
                                                    self.car_state['stop_flags']['manual_stop'] = False
                                except BlockingIOError:
                                    pass
                            
                            ## Car speed
                            if (self.car_state_history != self.car_state):
                                # print(self.car_state)
                                self.car_state_history = copy.deepcopy(self.car_state)
                                if (any(self.car_state['stop_flags'].values())):
                                    print('STOP', self.car_state)
                                    self.car.speedCtrl.stop()
                                else:
                                    print('START', self.car_state)
                                    self.car.speedCtrl.speed(my_lib.map(self.car_state['speed_limit'], 0,50,0,1))

                            # Show result with plots
                            if self.conf["DISPLAY"]["show_plots"]:
                                if self.objectDetector.drawed_img is not None:
                                    cv2.imshow("ObjectsDetector", cv2.cvtColor(self.objectDetector.drawed_img, cv2.COLOR_RGB2BGR))
                                if self.roadFollower.drawed_img is not None:
                                    cv2.imshow("RoadFollower", cv2.cvtColor(self.roadFollower.drawed_img, cv2.COLOR_RGB2BGR))
                            # Quit
                            key = cv2.waitKey(1)
                            if key == ord("q"):
                                break

                            
                            # elapsed_time = time.time() - start_time
                            # if (elapsed_time < self.min_execution_time):
                            #     time.sleep(self.min_execution_time - elapsed_time)
                            # self.threads_fps['Main'] = 1/(time.time()-start_time)
                            # start_time = time.time()

                            # Print thread FPS
                            if self.conf['DISPLAY']['show_fps']:
                                # os.system('clear')
                                row_format_str ="{:^20}" * (len(self.threads_fps))
                                row_format_num ="{:^20.1f}" * (len(self.threads_fps))
                                print(row_format_str.format(*self.threads_fps.keys()))
                                print(row_format_num.format(*self.threads_fps.values()))

                            

        cv2.destroyAllWindows()

if __name__ == '__main__':
    app = AutonomousCarApp(CONFIG_FNAME)
    app.start()



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

import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from datetime import datetime
import picamera
import picamera.array
import cv2
import yaml
import os.path
import time
from my_camera import PicameraController

SAVE_FOLDER = "/home/pi/Documents/AutonomousRcCar/Images/signLearningImg/"
IMG_NUM_START = 66


# Load configuration file
with open('conf.yaml') as fd:
    conf = yaml.load(fd, Loader=yaml.FullLoader)

camera = PicameraController(
            cam_param_dict = [(arg, value) for (arg, value) in conf['CAMERA']['parameters'].items() if value != None]
        )

def start():
    img_num = IMG_NUM_START
    if not os.path.isdir(SAVE_FOLDER):
        raise ValueError(f"'{SAVE_FOLDER}' is not an existing folder")
    with camera:
        while True:
            img = camera.current_frame

            # Show
            cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
            cv2.imshow("Original", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

            # Commands
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            elif key == ord("p"):
                filename = f"IMG{img_num}.jpg"
                cv2.imwrite(SAVE_FOLDER + filename , cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                img_num += 1
                print(f"Image {filename} save in {SAVE_FOLDER} directory")

    # Close properly
    cv2.destroyAllWindows()

if __name__ == '__main__':
    start()



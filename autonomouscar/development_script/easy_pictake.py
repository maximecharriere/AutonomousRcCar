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
import camera_calibration

SAVE_FOLDER = "/home/pi/Documents/AutonomousRcCar/Images/signLearningImg/"
IMG_NUM_START = 43


# Load configuration file
with open('conf.yaml') as fd:
    conf = yaml.load(fd, Loader=yaml.FullLoader)


def start():
    if not os.path.isdir(SAVE_FOLDER):
        raise ValueError(f"'{SAVE_FOLDER}' is not an existing folder")

    with picamera.PiCamera(resolution=conf["CAMERA"]["resolution"], sensor_mode=2) as camera:
        with picamera.array.PiRGBArray(camera, size=conf["CAMERA"]["resolution"]) as rawCapture:
            if not conf["CAMERA"]["awb"]: camera.awb_mode = 'off'
            if conf["CAMERA"]["awb_gains"]: camera.awb_gains = conf["CAMERA"]["awb_gains"]
            if conf["CAMERA"]["contrast"]: camera.contrast = conf["CAMERA"]["contrast"]
            if conf["CAMERA"]["saturation"]: camera.saturation = conf["CAMERA"]["saturation"]
            if conf["CAMERA"]["sharpness"]: camera.sharpness = conf["CAMERA"]["sharpness"]
            # Let time to the camera for color and exposure calibration
            time.sleep(1)

            img_num = IMG_NUM_START
            for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True): #we can directly take a bgr format, so we don't have to make a colorconversion when we want to show images with OpenCV, but RGB is the standard and deep learning model are trained with RGB images
                img = frame.array
                img = camera_calibration.undistort(
                    img = img, 
                    calParamFile = conf["IMAGE_PROCESSING"]["calibration"]["param_file"], 
                    crop = True)

                # Reset analised frame
                rawCapture.truncate(0)

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
            camera.stop_preview()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    start()



#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              imgChannelDifference.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    18.05.2020
#   Last modif date:   18.05.2020
# ----------------------------------- Infos -----------------------------------

# -------------------------------- Description --------------------------------
#   Show the camera stream in different colorspaces
# -------------------------------- Description --------------------------------

import sys
sys.path.append('..')
import time
import picamera
import picamera.array
import cv2
import matplotlib.pyplot as plt
from autonomouscar import camera_calibration, perspective_warp, my_lib

def main(argv):
    camResolution = (640, 480)
    perspectiveWarpPoints = [(173, 1952), (2560, 1952), (870, 920), (1835, 920)]
    perspectiveWarpPointsResolution = (2592, 1952)

    with picamera.PiCamera(resolution=camResolution) as camera:
        with picamera.array.PiRGBArray(camera, size=camResolution) as rawCapture:
            (bg, rg) = camera.awb_gains
            camera.awb_mode = 'off'
            camera.awb_gains = (111/128, 13/8)
            camera.contrast=50
            camera.saturation=100
            camera.sharpness=0
            time.sleep(1) # Let time to the camera for color and exposure calibration
            my_lib.print_caminfos(camera)

            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                frameBGR = frame.array
                frameBGR_calibrate = camera_calibration.undistort(
                    frameBGR, calParamFile="resources/cameraCalibrationParam_V2.pickle", crop=True)
                frameBGR_warped = perspective_warp.warp(frameBGR_calibrate, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameHSV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HSV)
                frameHLS = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HLS)
                frameLAB = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2LAB)
                frameLUV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2LUV)

                # Reset analised frame
                rawCapture.truncate(0)

                ## Show many images with plt
                fig, axs = plt.subplots(2, 1)
                axs[0].imshow(cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2RGB), cmap="gray")
                axs[0].set_title("Original")
                axs[1].imshow(cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2GRAY), cmap="gray")
                axs[1].set_title("Gray")
                fig.tight_layout()

                fig, axs = plt.subplots(3, 1)
                axs[0].imshow(my_lib.scaledSobelXY(frameHSV[:,:,0]))
                axs[0].set_title("HSV H channel")
                axs[1].imshow(my_lib.scaledSobelXY(frameHSV[:,:,1]))
                axs[1].set_title("HSV S channel")
                axs[2].imshow(my_lib.scaledSobelXY(frameHSV[:,:,2]))
                axs[2].set_title("HSV V channel")
                fig.tight_layout()
                fig, axs = plt.subplots(3, 1)
                axs[0].imshow(frameHLS[:,:,0], cmap="gray")
                axs[0].set_title("HLS H channel")
                axs[1].imshow(frameHLS[:,:,1], cmap="gray")
                axs[1].set_title("HLS L channel")
                axs[2].imshow(frameHLS[:,:,2], cmap="gray")
                axs[2].set_title("HLS S channel")
                fig.tight_layout()
                fig, axs = plt.subplots(3, 1)
                axs[0].imshow(frameLAB[:,:,0], cmap="gray")
                axs[0].set_title("LAB L channel")
                axs[1].imshow(frameLAB[:,:,1], cmap="gray")
                axs[1].set_title("LAB A channel")
                axs[2].imshow(frameLAB[:,:,2], cmap="gray")
                axs[2].set_title("LAB B channel")
                fig.tight_layout()
                fig, axs = plt.subplots(3, 1)
                axs[0].imshow(frameLUV[:,:,0], cmap="gray")
                axs[0].set_title("LUV L channel")
                axs[1].imshow(frameLUV[:,:,1], cmap="gray")
                axs[1].set_title("LUV U channel")
                axs[2].imshow(frameLUV[:,:,2], cmap="gray")
                axs[2].set_title("LUV V channel")
                fig.tight_layout()
                plt.show()

if __name__ == "__main__":
   main(sys.argv[1:])
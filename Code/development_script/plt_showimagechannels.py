#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
# ----------------------------------- Infos -----------------------------------

# -------------------------------- Description --------------------------------
#   Show the camera stream in different colorspaces
# -------------------------------- Description --------------------------------

import sys
sys.path.append('..')
import time
import picamera
import picamera.array
import numpy as np
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
            camera.awb_gains = (1, 211/128)
            camera.contrast=50
            camera.saturation=100
            camera.sharpness=0
            # Let time to the camera for color and exposure calibration
            time.sleep(1)
            my_lib.print_caminfos(camera)

            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                frameBGR = frame.array
                frameBGR_calibrate = camera_calibration.undistort(
                    frameBGR, calParamFile="resources/cameraCalibrationParam_V2.pickle", crop=True)
                frameGray              = cv2.cvtColor(frameBGR_calibrate, cv2.COLOR_BGR2GRAY)
                frameHSV               = cv2.cvtColor(frameBGR_calibrate, cv2.COLOR_BGR2HSV)
                frameHLS               = cv2.cvtColor(frameBGR_calibrate, cv2.COLOR_BGR2HLS)
                frameLAB               = cv2.cvtColor(frameBGR_calibrate, cv2.COLOR_BGR2LAB)
                frameLUV               = cv2.cvtColor(frameBGR_calibrate, cv2.COLOR_BGR2LUV)
                frameGray_sobel         = my_lib.sobelXY(frameGray)
                frameHSV_sobel         = my_lib.sobelXY(frameHSV) #we have to compute somel before we warp
                frameHLS_sobel         = my_lib.sobelXY(frameHLS)
                frameLAB_sobel         = my_lib.sobelXY(frameLAB)
                frameLUV_sobel         = my_lib.sobelXY(frameLUV)
                frameBGR_warped        = perspective_warp.warp(frameBGR_calibrate, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameGray_warped        = perspective_warp.warp(frameGray, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameHSV_warped        = perspective_warp.warp(frameHSV, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameHLS_warped        = perspective_warp.warp(frameHLS, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameLAB_warped        = perspective_warp.warp(frameLAB, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameLUV_warped        = perspective_warp.warp(frameLUV, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameGray_sobel_warped = perspective_warp.warp(frameGray_sobel, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameHSV_sobel_warped  = perspective_warp.warp(frameHSV_sobel, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameHLS_sobel_warped  = perspective_warp.warp(frameHLS_sobel, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameLAB_sobel_warped  = perspective_warp.warp(frameLAB_sobel, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                frameLUV_sobel_warped  = perspective_warp.warp(frameLUV_sobel, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
                for i in range(3):
                    frameHSV_sobel_warped[:,:,i]  = np.uint8(frameHSV_sobel_warped[:,:,i]/np.max(frameHSV_sobel_warped[:,:,i])*255)
                    frameHLS_sobel_warped[:,:,i]  = np.uint8(frameHLS_sobel_warped[:,:,i]/np.max(frameHLS_sobel_warped[:,:,i])*255)
                    frameLAB_sobel_warped[:,:,i]  = np.uint8(frameLAB_sobel_warped[:,:,i]/np.max(frameLAB_sobel_warped[:,:,i])*255)
                    frameLUV_sobel_warped[:,:,i]  = np.uint8(frameLUV_sobel_warped[:,:,i]/np.max(frameLUV_sobel_warped[:,:,i])*255)
                frameGray_sobel_warped = np.uint8(frameGray_sobel_warped/np.max(frameGray_sobel_warped)*255)
                # Reset analised frame
                rawCapture.truncate(0)

                ## Show many images with plt
                fig, axs = plt.subplots(3, 1)
                axs[0].imshow(cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2RGB), cmap="gray")
                axs[0].set_title("Original")
                axs[1].imshow(frameGray_warped, cmap="gray")
                axs[1].set_title("Gray")
                axs[2].imshow(frameGray_sobel_warped, cmap="gray")
                axs[2].set_title("GraySobel")
                fig.tight_layout()

                fig, axs = plt.subplots(3, 2)
                axs[0,0].imshow(frameHSV_warped[:,:,0], cmap="gray")
                axs[0,0].set_title("HSV H channel")
                axs[0,1].imshow(frameHSV_sobel_warped[:,:,0], cmap="gray")
                axs[0,1].set_title("Sobel")
                axs[1,0].imshow(frameHSV_warped[:,:,1], cmap="gray")
                axs[1,0].set_title("HSV S channel")
                axs[1,1].imshow(frameHSV_sobel_warped[:,:,1], cmap="gray")
                axs[1,1].set_title("Sobel")
                axs[2,0].imshow(frameHSV_warped[:,:,2], cmap="gray")
                axs[2,0].set_title("HSV V channel")
                axs[2,1].imshow(frameHSV_sobel_warped[:,:,2], cmap="gray")
                axs[2,1].set_title("Sobel")
                fig.tight_layout()

                fig, axs = plt.subplots(3, 2)
                axs[0,0].imshow(frameHLS_warped[:,:,0], cmap="gray")
                axs[0,0].set_title("HLS H channel")
                axs[0,1].imshow(frameHLS_sobel_warped[:,:,0], cmap="gray")
                axs[0,1].set_title("Sobel")
                axs[1,0].imshow(frameHLS_warped[:,:,1], cmap="gray")
                axs[1,0].set_title("HLS L channel")
                axs[1,1].imshow(frameHLS_sobel_warped[:,:,1], cmap="gray")
                axs[1,1].set_title("Sobel")
                axs[2,0].imshow(frameHLS_warped[:,:,2], cmap="gray")
                axs[2,0].set_title("HLS S channel")
                axs[2,1].imshow(frameHLS_sobel_warped[:,:,2], cmap="gray")
                axs[2,1].set_title("Sobel")
                fig.tight_layout()

                fig, axs = plt.subplots(3, 2)
                axs[0,0].imshow(frameLAB_warped[:,:,0], cmap="gray")
                axs[0,0].set_title("LAB L channel")
                axs[0,1].imshow(frameLAB_sobel_warped[:,:,0], cmap="gray")
                axs[0,1].set_title("Sobel")
                axs[1,0].imshow(frameLAB_warped[:,:,1], cmap="gray")
                axs[1,0].set_title("LAB A channel")
                axs[1,1].imshow(frameLAB_sobel_warped[:,:,1], cmap="gray")
                axs[1,1].set_title("Sobel")
                axs[2,0].imshow(frameLAB_warped[:,:,2], cmap="gray")
                axs[2,0].set_title("LAB B channel")
                axs[2,1].imshow(frameLAB_sobel_warped[:,:,2], cmap="gray")
                axs[2,1].set_title("Sobel")
                fig.tight_layout()

                fig, axs = plt.subplots(3, 2)
                axs[0,0].imshow(frameLUV_warped[:,:,0], cmap="gray")
                axs[0,0].set_title("LUV L channel")
                axs[0,1].imshow(frameLUV_sobel_warped[:,:,0], cmap="gray")
                axs[0,1].set_title("Sobel")
                axs[1,0].imshow(frameLUV_warped[:,:,1], cmap="gray")
                axs[1,0].set_title("LUV U channel")
                axs[1,1].imshow(frameLUV_sobel_warped[:,:,1], cmap="gray")
                axs[1,1].set_title("Sobel")
                axs[2,0].imshow(frameLUV_warped[:,:,2], cmap="gray")
                axs[2,0].set_title("LUV V channel")
                axs[2,1].imshow(frameLUV_sobel_warped[:,:,2], cmap="gray")
                axs[2,1].set_title("Sobel")
                fig.tight_layout()
                plt.show()

if __name__ == "__main__":
   main(sys.argv[1:])
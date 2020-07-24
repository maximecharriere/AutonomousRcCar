#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
# ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   Privide functions to get camera calibration parameters and to undistort
#   a picture with with parameters
## -------------------------------- Description --------------------------------

import cv2
import pickle


class ImgRectifier():
    def __init__(self, calParamFile, imgShape):
        try:
            with open(calParamFile, mode='rb') as fd:
                file = pickle.load(fd)    
                self.mtx = file['mtx']
                self.dist = file['dist']
                calImgShape = file['calImgShape']
                self.mtx_new = file['mtx_new']
                # Modify the mtx matrix if calibration images and image to distord are not the same shape
                if(imgShape != calImgShape): 
                    self.mtx[0,0] *= (imgShape[1] / calImgShape[1]) #fx
                    self.mtx[1,1] *= (imgShape[0] / calImgShape[0]) #fy
                    self.mtx[0,2] *= (imgShape[1] / calImgShape[1]) #cx
                    self.mtx[1,2] *= (imgShape[0] / calImgShape[0]) #cy

                    self.mtx_new[0,0] *= (imgShape[1] / calImgShape[1]) #fx
                    self.mtx_new[1,1] *= (imgShape[0] / calImgShape[0]) #fy
                    self.mtx_new[0,2] *= (imgShape[1] / calImgShape[1]) #cx
                    self.mtx_new[1,2] *= (imgShape[0] / calImgShape[0]) #cy
                    
        except FileNotFoundError:
            raise FileNotFoundError("pickle file with calibration parameters \
                not found. See 'camera_rectifier_generator.py' to \
                generate the file.")

    def undistort(self, img, crop = True):
        return cv2.undistort(img, self.mtx, self.dist, None, None if crop else self.mtx_new)
#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              cameraCalibration.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    18.04.2020
#   Last modif date:   04.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   Privide functions to get camera calibration parameters and to undistort
#   a picture with with parameters
## -------------------------------- Description --------------------------------


import numpy as np
import cv2
import glob
import pickle
import matplotlib.pyplot as plt


def getCameraCalibrationParameters(nRow, nCol, imgDirectory, calParamFile=None, saveDrawedImg=False):
    # prepare real world point as [x,y,z] coordinate. Z always on the same plane, so it keep 0. 
    # XY are (0,0),(1,0),(2,0),...,(8,0), (0,1),(1,1),...,(8,1), (0,2),......,(8,5)
    objp = np.zeros((nRow*nCol,3), np.float32)
    objp[:,:2] = np.mgrid[0:nCol,0:nRow].T.reshape(-1,2)

    # Arrays to store object points (real world) and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # Calibration images folder
    imgDirectory = imgDirectory+"/*.jpg"
    images = glob.glob(imgDirectory)
    if (len(images)<1):
        raise ValueError("No file found")
    
    # Find chestboard corners in all images
    for i, fname in enumerate(images):
        # Load image
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (nCol,nRow),None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            # Refines the corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Max 30 iter and epsilon of 0.001
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
            imgpoints.append(corners2)
            # Draw and save the corners in new image
            if saveDrawedImg:
                drawedImg = cv2.drawChessboardCorners(img, (nCol,nRow), corners, ret)
                cv2.imwrite(fname.replace(".jpg","_findedCorners.jpg"), drawedImg) 
        else:
            print(f"Corners not found in img {fname}")
        # print progression
        print(f"Image {i+1}/{len(images)} analysed")
    
    # Calibrate the camera
    img_size = (img.shape[1], img.shape[0])
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None,None)
    mtx_new, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,img_size, 1)

    # Save camera calibration for later use
    if calParamFile is not None:
        calParamPickle = {}
        calParamPickle['mtx'] = mtx
        calParamPickle['dist'] = dist
        calParamPickle['mtx_new'] = mtx_new
        calParamPickle['calImgShape'] = img.shape
        pickle.dump(calParamPickle, open(calParamFile, 'wb') )

    return mtx, dist


def undistort(img, calParamFile, crop = True):
    try:
        with open(calParamFile, mode='rb') as fd:
            file = pickle.load(fd)    
            mtx = file['mtx']
            dist = file['dist']
            calImgShape = file['calImgShape']
            mtx_new = None
            if not crop:
                mtx_new = file['mtx_new']
            # Modify the mtx matrix if calibration images and image to distord are not the same shape
            if(img.shape != calImgShape):
                mtx[0,0] *= (img.shape[1] / calImgShape[1]) #fx
                mtx[1,1] *= (img.shape[0] / calImgShape[0]) #fy
                mtx[0,2] *= (img.shape[1] / calImgShape[1]) #cx
                mtx[1,2] *= (img.shape[0] / calImgShape[0]) #cy
                if mtx_new is not None:
                    mtx_new[0,0] *= (img.shape[1] / calImgShape[1]) #fx
                    mtx_new[1,1] *= (img.shape[0] / calImgShape[0]) #fy
                    mtx_new[0,2] *= (img.shape[1] / calImgShape[1]) #cx
                    mtx_new[1,2] *= (img.shape[0] / calImgShape[0]) #cy
            return cv2.undistort(img, mtx, dist, None, mtx_new)
    except FileNotFoundError:
        raise


# getCameraCalibrationParameters(5, 8, "/home/pi/Documents/AutonomousRcCar/Code/CameraCalibration/ImagesV2", calParamFile="/home/pi/Documents/AutonomousRcCar/Code/CameraCalibration/cameraCalibrationParam_V2.pickle", saveDrawedImg=False)

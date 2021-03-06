#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
# ----------------------------------- Infos -----------------------------------

# -------------------------------- Description --------------------------------
#   Script to generate the camera matrix and distorsion coeficients 
#   to distord an image
# -------------------------------- Description --------------------------------

# ------------------------------------ Use ------------------------------------
#   usage: camera_rectifier_generator.py [-h] --pattern_size PATTERN_SIZE
#                                      --img_directory IMG_DIRECTORY
#                                      [--output_fname OUTPUT_FNAME]
#                                      [--save_drawing SAVE_DRAWING]
# 
#   Creat a file with camera rectifier parameters
#   
#   optional arguments:
#     -h, --help            show this help message and exit
#     --pattern_size PATTERN_SIZE, -ps PATTERN_SIZE
#                           Number of inner corners per a chessboard row and
#                           column
#     --img_directory IMG_DIRECTORY, -i IMG_DIRECTORY
#                           Directory where calibration images are stored
#     --output_fname OUTPUT_FNAME, -o OUTPUT_FNAME
#                           Filename where to store camera calibration parameters.
#                           If None, no file is saved.
#     --save_drawing SAVE_DRAWING, -s SAVE_DRAWING
#                           Save a copy of the input images with finded chessboard
#                           drawed
# ------------------------------------ Use ------------------------------------

import os
import numpy as np
import cv2
import glob
import pickle
import argparse

def getCameraCalibrationParameters(patternSize, imgDirectory, calParamFile=None, saveDrawedImg=False):
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
        ret, corners = cv2.findChessboardCorners(gray, patternSize,None)
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

    return mtx, mtx_new, dist

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creat a file with camera rectifier parameters')
    parser.add_argument('--pattern_size', '-ps', type=tuple, required=True, help='Number of inner corners per a chessboard row and column')
    parser.add_argument('--img_directory', '-i', type=str, required=True, help='Directory where calibration images are stored')
    parser.add_argument('--output_fname', '-o', type=str, default=None, help='Filename where to store camera calibration parameters. If None, no file is saved.')
    parser.add_argument('--save_drawing', '-s', type=bool, default=False, help='Save a copy of the input images with finded chessboard drawed')

    args = parser.parse_args()

    getCameraCalibrationParameters(
        patternSize   = args.pattern_size, 
        imgDirectory  = args.img_directory, 
        calParamFile  = args.output_fname, 
        saveDrawedImg = args.save_drawing)
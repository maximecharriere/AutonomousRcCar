#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              myLib.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    12.04.2020
#   Last modif date:   12.04.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   This file contains all my useful functions that I often use 
## -------------------------------- Description --------------------------------


## Work exactly like Arduino's map function
# https://www.arduino.cc/reference/en/language/functions/math/map/

import matplotlib.pyplot as plt
import cv2
import numpy as np
import yaml

def load_configuration(conf_file):
    # Load configuration file
    with open(conf_file) as fd:
        conf = yaml.load(fd, Loader=yaml.FullLoader)
    return conf

def map(x_in, in_min, in_max, out_min, out_max, limit=False):
    x_out = (x_in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if limit:
        return clamp(x_out, out_min, out_max)
    else:
        return x_out

def mix(val1, val2, val1_proportion):
    val2_proportion = 1-val1_proportion
    return (val1*val1_proportion + val2*val2_proportion)

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

def print_caminfos (camera):
    print("Camera infos:")
    print(f"Brightness (get/set):            {camera.brightness}")
    print(f"Exposure Mode (get/set):         {camera.exposure_mode}")
    print(f"Shutter Speed (get/set):         {camera.shutter_speed}")
    print(f"Exposure Speed (get):            {camera.exposure_speed}")
    print(f"ISO (get/set):                   {camera.iso}")
    print(f"Analog Gain (get):               {camera.analog_gain}")
    print(f"Digital Gain (get):              {camera.digital_gain}")
    print(f"Contrast (get/set):              {camera.contrast}")
    print(f"Saturation (get/set):            {camera.saturation}")
    print(f"Sharpness (get/set):             {camera.sharpness}")
    print(f"Exposure Compensation (get/set): {camera.exposure_compensation}")
    print(f"AWB Mode (get/set):              {camera.awb_mode}")
    print(f"AWB Gain (get/set):              {camera.awb_gains}")
    print(f"Color Effects (get/set):         {camera.color_effects}")
    print(f"DRC Strength (get/set):          {camera.drc_strength}")
    print(f"Resolution (get/set):            {camera.resolution}")
    print(f"Framerate (get/set):             {camera.framerate}")
    print(f"Sensor Mode (get/set):           {camera.sensor_mode}")

def plt_show_all_colorspace(img):
    '''img must be BGR'''
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frameLAB = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    frameLUV = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
    
    ## Show images with plt
    fig, axs = plt.subplots(3, 3)
    axs[0,0].imshow(frameHSV[0], cmap="gray")
    axs[0,0].set_title("HSV H channel")
    axs[0,1].imshow(frameHSV[1], cmap="gray")
    axs[0,1].set_title("HSV S channel")
    axs[0,2].imshow(frameHSV[2], cmap="gray")
    axs[0,2].set_title("HSV V channel")
    axs[1,0].imshow(frameLAB[0], cmap="gray")
    axs[1,0].set_title("LAB L channel")
    axs[1,1].imshow(frameLAB[1], cmap="gray")
    axs[1,1].set_title("LAB A channel")
    axs[1,2].imshow(frameLAB[2], cmap="gray")
    axs[1,2].set_title("LAB B channel")
    axs[2,0].imshow(frameLUV[0], cmap="gray")
    axs[2,0].set_title("LUV L channel")
    axs[2,1].imshow(frameLUV[1], cmap="gray")
    axs[2,1].set_title("LUV U channel")
    axs[2,2].imshow(frameLUV[2], cmap="gray")
    axs[2,2].set_title("LUV V channel")

    plt.show()

def inRangeHSV(src, lowerb, upperb):
    lowerb = np.array(lowerb)
    upperb = np.array(upperb)
    if not (len(src.shape) == 3):
        raise ValueError("The source must be a 3D matrix")
    if not (src.shape[2] == len(lowerb) == len(upperb) == 3):
        raise ValueError("Parameters must have 3 channels")
    
    if (lowerb[0] <= upperb[0]):
        return cv2.inRange(src, lowerb, upperb)
    else:
        h_max = 180
        h_min = 0
        mask1 = cv2.inRange(src, lowerb, (h_max, upperb[1], upperb[2]))
        mask2 = cv2.inRange(src, (h_min, lowerb[1], lowerb[2]), upperb)
        return mask1 | mask2

def scaledSobelXY(img):
    derivative = sobelXY(img)
    derivative_scaled = np.uint8(derivative/np.max(derivative)*255)
    return derivative_scaled

def sobelXY(img):
    derivativeX_abs = np.absolute(cv2.Scharr(img, cv2.CV_16S, 1, 0))  # Take the s channel derivative in x, CV_16S to have neg value
    derivativeY_abs = np.absolute(cv2.Scharr(img, cv2.CV_16S, 0, 1))  # Take the s channel derivative in y
    derivative =derivativeX_abs + derivativeY_abs
    return derivative

def isNaN(num):
    return num != num

def isaN(num):
    return not isNaN(num)
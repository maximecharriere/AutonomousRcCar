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

import picamera

def Map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def PrintCamInfos ( camera):
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
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

# Add parent dir to sys.path
import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

import cv2
import yaml
import os.path
import time
from camera_controller import PicameraController
from my_lib import load_configuration
import argparse

def start(save_folder, start_num=0):
    # Load configuration file
    conf_fname = os.path.join(parentdir, 'conf.yaml')
    conf = load_configuration(conf_fname)

    camera = PicameraController(
        cam_param_dict = [(arg, value) for (arg, value) in conf['CAMERA']['parameters'].items() if value != None])
    
    img_num = start_num
    if not os.path.isdir(save_folder):
        raise ValueError(f"'{save_folder}' is not an existing folder")
    with camera:
        print("----- Commands -----")
        print("p: Capture img")
        print("q: Quit")

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
                cv2.imwrite(os.path.join(save_folder, filename) , cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                img_num += 1
                print(f"Image {filename} save in {save_folder} directory")

    # Close properly
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Takes numbered pictures when the "p" key is pressed')
    parser.add_argument('--output_folder', '-o', type=str, required=True, help='Path to the folder there images have to be save')
    parser.add_argument('--start_num', '-s', type=int, required=False, default=0, help='Number of the first image')
    args = parser.parse_args()

    start(
        save_folder=args.output_folder,
        start_num=args.start_num)



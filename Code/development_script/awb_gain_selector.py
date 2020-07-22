#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
# ----------------------------------- Infos -----------------------------------

# -------------------------------- Description --------------------------------
#   Script for manual camera white balance calibration
# -------------------------------- Description --------------------------------

# ------------------------------------ Use ------------------------------------
#   usage: awb_gain_selector.py [-h]
# ------------------------------------ Use ------------------------------------


# Add parent dir to sys.path
import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

import time
import pygame
from camera_controller import PicameraController
from my_lib import load_configuration
import argparse

def main():
    step = 1/100
    # Load configuration file
    conf_fname = os.path.join(parentdir, 'conf.yaml')
    conf = load_configuration(conf_fname)
    camera = PicameraController(
            cam_param_dict = [(arg, value) for (arg, value) in conf['CAMERA']['parameters'].items() if value != None])
    
    pygame.init()
    display = pygame.display.set_mode(camera.resolution)
    pygame.display.set_caption('AWB Selector')

    print("Commands:")
    print("  UP:    Add blue")
    print("  DOWN:  Remove blue")
    print("  RIGHT: Add red")
    print("  LEFT:  Remove red")

    with camera:
        # Disable awb to manualy correct it
        (rg, bg) =  camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = (rg, bg)
        while True:
            surface = pygame.pixelcopy.make_surface(camera.current_frame.transpose((1,0,2)))
            display.blit(surface, (0,0))
            pygame.display.update()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        bg += step
                    elif event.key == pygame.K_DOWN:
                        bg -= step
                    elif event.key == pygame.K_RIGHT:
                        rg += step
                    elif event.key == pygame.K_LEFT:
                        rg -= step
                    camera.awb_gains = (rg, bg)
                    print(f"Red: {camera.awb_gains[0]}    Blue: {camera.awb_gains[1]}")

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script for manual camera white balance calibration')
    args = parser.parse_args()

    main()
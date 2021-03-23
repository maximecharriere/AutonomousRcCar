#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              manualDrive.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    20.03.2020
#   Last modif date:   01.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   Read inputs from controller to drive the car
## -------------------------------- Description --------------------------------

import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
import asyncio
from evdev import InputDevice, categorize, ecodes, util
import my_lib
from actuator_controller import SteeringController, SpeedController

"""Pin declaration with BCM format"""
PIN_SPEED = 18
PIN_STEERING = 19

def print_help():
    print(f"{os.path.basename(__file__)} -e <Controller event filename>")

def main(argv):
    event_filename = '/dev/input/event0'
    #Get arguments
    try:
       opts, args = getopt.getopt(argv, "h?e:", ["help", "event="])
    except getopt.GetoptError:
       print_help()
       sys.exit(2)
    for opt, arg in opts:
       if opt in ('-h', '-?', '--help'):
          print_help()
          sys.exit()
       elif opt in ("-e", "--event"):
           event_filename = arg
    run_manually(event_filename)


def run_manually(event_filename):
    controller = InputDevice(event_filename)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(event_manager(controller))
    loop.run_in_executor
async def event_manager(device):
    SpeedCtrl = SpeedController(PIN_SPEED,5.5,9.5)
    SteeringCtrl = SteeringController(PIN_STEERING,5.5,9.5)
    async for event in device.async_read_loop():
        if event.type == ecodes.EV_ABS:
            if  event.code == ecodes.ABS_X:  #Joy Gauche / Gauche- Droite+
                SteeringCtrl.Angle(my_lib.map(event.value, 0, 2**16, 0, 100))
                print("X: ", event.value)
            elif  event.code == ecodes.ABS_Y: #Joy Gauche / Haut- Bas+
                SpeedCtrl.Speed(my_lib.map(event.value, 0, 2**16, 30, 60))
                print("Y: ", event.value)
        #    elif event.code == ecodes.ABS_RX: #Joy Droit / Gauche- Droite+
        #        print("Joy Droit / Gauche- Droite+")
        #    elif  event.code == ecodes.ABS_RY: #Joy Droit / Haut- Bas+
        #        print("Joy Droit / Haut- Bas+")
        #    elif  event.code == ecodes.ABS_RZ: #Gachette droite
        #         print("Gachette droite")
        #    elif  event.code == ecodes.ABS_Z: #Gachette gauche
        #         print("Gachette gauche")
        #    elif  event.code == ecodes.ABS_HAT0X:
        #        if event.value == 1: #Croix / Droite
        #            print("Croix / Droite")
        #        elif  event.value == -1: #Croix / Gauche
        #            print("Croix / Gauche")
        #    elif  event.code == ecodes.ABS_HAT0Y:
        #        if event.value == 1: #Croix / Bas
        #            print("Croix / Bas")
        #        elif  event.value == -1: #Croix / Haut
        #            print("Croix / Haut")
        #elif event.type == ecodes.EV_KEY:
        #    if event.value == True:
        #        if event.code == ecodes.BTN_X:
        #            print("Bouton X")
        #        elif  event.code == ecodes.BTN_Y:
        #            print("Bouton Y")
        #        elif  event.code == ecodes.BTN_B:
        #            print("Bouton B")
        #        elif  event.code == ecodes.BTN_A:
        #            print("Bouton A")
        #        elif  event.code == ecodes.BTN_THUMBR:
        #            print("Bouton Joy R")
        #        elif  event.code == ecodes.BTN_THUMBL:
        #            print("Bouton Joy L")
        #        elif  event.code == ecodes.BTN_START:
        #            print("Bouton Menu")
        #        elif  event.code == ecodes.BTN_SELECT:
        #            print("Bouton Double Fenetre")
        #        elif  event.code == ecodes.BTN_MODE:
        #            print("Bouton Power")
        #        elif  event.code == ecodes.BTN_TR:
        #            print("Bouton gachette droite")
        #        elif  event.code == ecodes.BTN_TL:
        #            print("Bouton gachette gauche")


if __name__ == "__main__":
   main(sys.argv[1:])
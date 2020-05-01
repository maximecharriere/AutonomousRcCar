#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              printControllerInputs.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    12.04.2020
#   Last modif date:   01.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   This file print the input of a controller in the terminal.
#   The file containing the events of the controller have to be passed when this
#   script is called with the following argument
#   -f <Controller event filename>
#
#   The event file is in the /dev/input/ directory
## -------------------------------- Description --------------------------------

import sys, getopt, os
import asyncio
from evdev import InputDevice, categorize, ecodes ,util

# Function to manage the events raised by the controller
async def EventManager(device):
    async for event in device.async_read_loop():
        if event.type == ecodes.EV_KEY:
            if event.value == True:
                print(f"Key: {ecodes.keys[event.code]}")
        elif event.type == ecodes.EV_ABS:
            print(f"Joystick: {ecodes.ABS[event.code]}    Value: {event.value}")

def main(argv):
    ControllerEventFile = InputDevice('/dev/input/event0')

    #Get arguments
    try:
       opts, args = getopt.getopt(argv,"h?f:",["help","file="])
    except getopt.GetoptError:
       print(f"{os.path.basename(__file__)} -f <Controller event filename>")
       sys.exit(2)
    for opt, arg in opts:
       if opt in ('-h', '-?', '--help'):
          print(f"{os.path.basename(__file__)} -f <Controller event filename>")
          sys.exit()
       elif opt in ("-f", "--file"):
          ControllerEventFile = InputDevice(arg)
            
    # Run the EventManager
    loop = asyncio.get_event_loop()
    loop.run_until_complete(EventManager(ControllerEventFile))

if __name__ == "__main__":
   main(sys.argv[1:])
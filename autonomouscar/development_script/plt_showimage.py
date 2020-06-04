#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              pltShowImage.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    30.05.2020
#   Last modif date:   01.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   Simply displays the image passed as a parameter in a plt window
## -------------------------------- Description --------------------------------

import sys, os, getopt
import numpy as np
import cv2
import matplotlib.pyplot as plt

def plt_show(filename):
   assert os.path.isfile(filename), "Image not found"
   img = cv2.imread(filename)
   img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
   plt.imshow(img,cmap = 'gray')
   plt.show()


def main(argv):
    #Get arguments
    try:
       opts, args = getopt.getopt(argv, "h?i:", ["help", "image="])
    except getopt.GetoptError:
       print(f"{os.path.basename(__file__)} -i <Image filename>")
       sys.exit(2)
    for opt, arg in opts:
       if opt in ('-h', '-?', '--help'):
          print(f"{os.path.basename(__file__)} -i <Image filename>")
          sys.exit()
       elif opt in ("-i", "--image"):
          plt_show(arg)

if __name__ == "__main__":
   main(sys.argv[1:])
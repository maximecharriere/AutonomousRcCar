#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              PwmController.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    12.04.2020
#   Last modif date:   01.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   This file define the classes to control the speed and the steering of the
#   RC car
## -------------------------------- Description --------------------------------

import RPi.GPIO as GPIO
import myLib

class PwmController:
    def __init__(self, pin, minPercent, maxPercent):
        self.PWM_FREQ = 50
        self.Pin = pin
        self.MinPercent = minPercent
        self.MaxPercent = maxPercent
        self.NeutralPercent = (maxPercent+minPercent)/2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin,GPIO.OUT)
        self.PwmObj = GPIO.PWM(pin , self.PWM_FREQ)
        self.PwmObj.start(self.NeutralPercent)

class SteeringController(PwmController):
    def __init__(self, pin, minPercent, maxPercent):
        PwmController.__init__(self,pin, minPercent, maxPercent)

    def Forward(self):
        self.PwmObj.ChangeDutyCycle(self.NeutralPercent)

    def Angle(self, percent):
        """Set the current wheel angle
        0   = MAX LEFT
        50  = FORWARD
        100 = MAX RIGHT"""
        self.PwmObj.ChangeDutyCycle(myLib.Map(percent,0,100,self.MaxPercent,self.MinPercent))


class SpeedController(PwmController):
    def __init__(self, pin, minPercent, maxPercent):
        PwmController.__init__(self,pin, minPercent, maxPercent)

    def Stop(self):
        self.PwmObj.ChangeDutyCycle(self.NeutralPercent)

    def Speed(self, percent):
        """Set the actual speed of the car
        0   = MAX SPEED BACKWARD
        50  = STOP
        100 = MAX SPEED FORWARD"""
        self.PwmObj.ChangeDutyCycle(myLib.Map(percent,0,100,self.MaxPercent,self.MinPercent))


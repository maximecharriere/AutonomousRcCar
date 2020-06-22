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

from hardpwm import HardPWM
import my_lib
import time

class _PwmController:
    def __init__(self, pin, minDutyCycle, maxDutyCycle):
        '''
            pin: pin number of the PWM output given with the BCM format (generaly 18 and 19)
            minDutyCycle: time that the edge have to be 1 to be at the min position
            maxDutyCycle: time that the edge have to be 1 to be at the max position
        '''
        self.PWM_FREQ = 50
        self.Pin = pin
        self.MinDutyCycle = minDutyCycle
        self.MaxDutyCycle = maxDutyCycle
        self.NeutralDutyCycle = (minDutyCycle+maxDutyCycle)/2
        self.DutyCycle = self.NeutralDutyCycle
        self.PwmObj = HardPWM(pin)
        self.PwmObj.set_frequency(self.PWM_FREQ)
        self.PwmObj.set_duty_cycle(self.DutyCycle)
        self.PwmObj.enable()  

class SteeringController(_PwmController):
    def __init__(self, pin, minDutyCycle, maxDutyCycle):
        _PwmController.__init__(self,pin, minDutyCycle, maxDutyCycle)

    def Angle(self, angle):
        """Set the current wheel angle
        -1   = MAX LEFT
        0  = FORWARD
        1 = MAX RIGHT"""
        self.DutyCycle = my_lib.map(angle,-1,1,self.MinDutyCycle,self.MaxDutyCycle,limit=True)
        self.PwmObj.set_duty_cycle(self.DutyCycle)


class SpeedController(_PwmController):
    def __init__(self, pin, minDutyCycle, maxDutyCycle):
        _PwmController.__init__(self,pin, minDutyCycle, maxDutyCycle)

    def Stop(self):
        if (self.DutyCycle > self.NeutralDutyCycle): #if the car go forward, do an emergency stop
            self.PwmObj.set_duty_cycle(1)
            time.sleep(0.5)
        self.DutyCycle = self.NeutralDutyCycle
        self.PwmObj.set_duty_cycle(self.DutyCycle)

    def Speed(self, speed):
        """Set the actual speed of the car
        -1   = MAX SPEED BACKWARD
        0  = STOP
        1 = MAX SPEED FORWARD"""
        self.DutyCycle = my_lib.map(speed,-1,1,self.MinDutyCycle,self.MaxDutyCycle,limit=True)
        self.PwmObj.set_duty_cycle(self.DutyCycle)


if __name__ == "__main__":
    # Test code
    from time import sleep
    import numpy as np

    print("Testing of steering controller")
    SteeringCtrl = SteeringController(pin=19, minDutyCycle=1.0, maxDutyCycle=2.0)
    print("right > left")
    for angle in np.linspace(-1, 1, num=100, endpoint=True):
        SteeringCtrl.Angle(angle)
        print(f"    {angle}")
        sleep(0.05)
    print("\nleft > right")
    for angle in np.linspace(1, -1, num=100, endpoint=True):
        SteeringCtrl.Angle(angle)
        print(f"    {angle}")
        sleep(0.05)
    print("\nForward")
    SteeringCtrl.Angle(0)

    print("\n\nTesting of speed controller")
    for i in range(5):
        print("!!!!! WARNING : THE CAR GOING TO MOVE FAST !!!!!")
        sleep(1)
    SpeedCtrl = SpeedController(pin=18, minDutyCycle=1.0, maxDutyCycle=2.0)
    print("Forward")
    for speed in np.linspace(0, 1, num=50, endpoint=True):
        SpeedCtrl.Speed(speed)
        sleep(0.05)
    print("\nStop")
    SpeedCtrl.Stop()
    print("\nBackward")
    for speed in np.linspace(0, -1, num=50, endpoint=True):
        SpeedCtrl.Speed(speed)
        sleep(0.05)
    print("\nStop")
    SpeedCtrl.Stop()
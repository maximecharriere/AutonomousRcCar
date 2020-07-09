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

import my_lib, my_pwm
import time

PWM_FREQ = 50

class _PwmActuator(object):
    def __init__(self, pin, minDutyCycle, maxDutyCycle, hardware):
        '''
            pin (int): pin number of the PWM output given with the BCM format (generaly 18 and 19)
            minDutyCycle (double): time that the edge have to be 1 to be at the min position (in ms)
            maxDutyCycle (double): time that the edge have to be 1 to be at the max position (in ms)
            hardware (bool): generate a hardware or software pwm
        '''
        self.MinDutyCycle = minDutyCycle
        self.MaxDutyCycle = maxDutyCycle
        self.NeutralDutyCycle = (minDutyCycle+maxDutyCycle)/2
        if (hardware):
            self.pwm_ctrl= my_pwm.HardPwm(pin, PWM_FREQ)
        else:
            self.pwm_ctrl= my_pwm.SoftPwm(pin, PWM_FREQ)

        self.pwm_ctrl.set_duty_cycle(self.NeutralDutyCycle)

    def startPwm(self):
        self.pwm_ctrl.enable() 

    def stopPwm(self):
        self.pwm_ctrl.disable()


class SteeringController(_PwmActuator):
    def __init__(self, pin, minDutyCycle, maxDutyCycle, hardware=False):
        _PwmActuator.__init__(self,pin, minDutyCycle, maxDutyCycle, hardware)

    def angle(self, angle):
        """Set the current wheel angle
        -1   = MAX LEFT
        0  = FORWARD
        1 = MAX RIGHT"""
        self.pwm_ctrl.set_duty_cycle(my_lib.map(angle,-1,1,self.MinDutyCycle,self.MaxDutyCycle,limit=True))


class SpeedController(_PwmActuator):
    def __init__(self, pin, minDutyCycle, maxDutyCycle, hardware=False):
        _PwmActuator.__init__(self,pin, minDutyCycle, maxDutyCycle, hardware)

    def stop(self):
        #on my car, if it goes forward and I put the min dutycycle on the motor controller,
        #the car don't go backward, but do and emergency stop
        if (self.pwm_ctrl.duty_cycle > self.NeutralDutyCycle):
            self.pwm_ctrl.set_duty_cycle(self.minDutyCycle)
            time.sleep(0.3) #wait the car to be stopped
        self.pwm_ctrl.set_duty_cycle(self.NeutralDutyCycle)

    def speed(self, speed):
        """Set the actual speed of the car
        -1   = MAX SPEED BACKWARD
        0  = STOP
        1 = MAX SPEED FORWARD"""
        self.pwm_ctrl.set_duty_cycle(my_lib.map(speed,-1,1,self.MinDutyCycle,self.MaxDutyCycle,limit=True))



if __name__ == "__main__":
    # Test code
    from time import sleep
    import numpy as np

    print("Testing of steering controller")
    SteeringCtrl = SteeringController(pin=19, minDutyCycle=1.2, maxDutyCycle=1.8, hardware=True)
    print("right > left")
    for angle in np.linspace(-1, 1, num=100, endpoint=True):
        SteeringCtrl.angle(angle)
        print(f"    {angle}")
        sleep(0.05)
    print("\nleft > right")
    for angle in np.linspace(1, -1, num=100, endpoint=True):
        SteeringCtrl.angle(angle)
        print(f"    {angle}")
        sleep(0.05)
    print("\nForward")
    SteeringCtrl.angle(0)

    print("\n\nTesting of speed controller")
    for i in range(5):
        print("!!!!! WARNING : THE CAR GOING TO MOVE FAST !!!!!")
        sleep(1)
    SpeedCtrl = SpeedController(pin=18, minDutyCycle=1.2, maxDutyCycle=1.8, hardware=True)
    print("Forward")
    for speed in np.linspace(0, 1, num=50, endpoint=True):
        SpeedCtrl.speed(speed)
        print(f"    {speed}")
        sleep(0.05)
    print("\nStop")
    SpeedCtrl.stop()
    print("\nBackward")
    for speed in np.linspace(0, -1, num=50, endpoint=True):
        SpeedCtrl.speed(speed)
        print(f"    {speed}")
        sleep(0.05)
    print("\nStop")
    SpeedCtrl.stop()
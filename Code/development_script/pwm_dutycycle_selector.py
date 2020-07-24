#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
# ----------------------------------- Infos -----------------------------------

import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from my_pwm import HardPwm

PWM_PIN = 18
DUTYCYCLE_STEP = 0.005
DEFAUT_DUTYCYCLE = 1.6
PWM_FREQ = 50

def main(argv):
    pwm_obj = HardPwm(PWM_PIN, PWM_FREQ)
    pwm_obj.set_duty_cycle(DEFAUT_DUTYCYCLE)
    pwm_obj.enable() 
    
    print("Commands:")
    print("  w:  Increase duty cycle")
    print("  s:  Decrease duty cycle")
    print("  m:  Manual duty cycle")
    print("  q:  Quit")

    dutycycle = DEFAUT_DUTYCYCLE
    while True:
        char = input()
        if char == 'q':
            return
        elif char == 'w':
            dutycycle += DUTYCYCLE_STEP
        elif char == 's':
            dutycycle -= DUTYCYCLE_STEP
        elif char == 'm':
            dutycycle = float(input("Dutycycle: "))

        pwm_obj.set_duty_cycle(dutycycle)
        print(f"Dutycycle: {dutycycle}")

  

if __name__ == "__main__":
   main(sys.argv[1:])

import sys
import RPi.GPIO as GPIO

PWM_PIN = 18
DUTYCYCLE_STEP = 0.1
DEFAUT_DUTYCYCLE = 7.5
PWM_FREQ = 50

def main(argv):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PWM_PIN, GPIO.OUT)
    pwm_obj = GPIO.PWM(PWM_PIN , PWM_FREQ)
    pwm_obj.start(DEFAUT_DUTYCYCLE)
    
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

        pwm_obj.ChangeDutyCycle(dutycycle)
        print(f"Dutycycle: {dutycycle}")

  

if __name__ == "__main__":
   main(sys.argv[1:])

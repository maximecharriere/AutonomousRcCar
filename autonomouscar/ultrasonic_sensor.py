#Libraries
import RPi.GPIO as GPIO
import time

SOUND_SPEED = 343 #m/s

class UltrasonicSensor():
    def __init__(self, pin_trigger, pin_echo):
        self.pin_trigger = pin_trigger
        self.pin_echo = pin_echo
        #set GPIO direction (IN / OUT)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_trigger, GPIO.OUT)
        GPIO.setup(self.pin_echo, GPIO.IN)

    def __del__(self):
        GPIO.cleanup()


    def getDistance(self):
        # set Trigger to HIGH
        GPIO.output(self.pin_trigger, True)

        # set Trigger after 10us to LOW
        time.sleep(0.00001)
        GPIO.output(self.pin_trigger, False)

        # save StartTime
        start_time = time.time()
        while GPIO.input(self.pin_echo) == 0:
            start_time = time.time()
    
        # save time of arrival
        stop_time = time.time()
        while GPIO.input(self.pin_echo) == 1:
            stop_time = time.time()

        # time difference between start and arrival
        time_elapsed = stop_time - start_time
        # multiply with the sonic speed
        # and divide by 2, because there and back
        distance = (time_elapsed * SOUND_SPEED) / 2
        return distance
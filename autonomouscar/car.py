from actuator_controller import SteeringController, SpeedController
from my_camera import PicameraController
from ultrasonic_sensor_controller import UltrasonicSensor
import time 
class Car():
    def __init__(self, conf):
        self.speedCtrl = SpeedController(
            pin = conf["PIN"]["pwm_speed"], 
            minDutyCycle = conf["CAR"]["speed_pwm_dc_min"],
            maxDutyCycle = conf["CAR"]["speed_pwm_dc_max"], 
            hardware=True)
        self.steeringCtrl = SteeringController(
            pin = conf["PIN"]["pwm_steering"], 
            minDutyCycle = conf["CAR"]["steering_pwm_dc_min"],
            maxDutyCycle = conf["CAR"]["steering_pwm_dc_max"], 
            hardware=True)
        self.camera = PicameraController(
            cam_param_dict = [(arg, value) for (arg, value) in conf['CAMERA']['parameters'].items() if value != None]
        )
        self.ultrasonicSensor = UltrasonicSensor(
            pin_trigger = conf['PIN']['proximity_trigger'],
            pin_echo = conf['PIN']['proximity_echo']
        )
        
    def __enter__(self):
        """ Entering a with statement """
        self.start()
        return self
    
    def __exit__(self, exception_type, exception_value, traceback):
        self.stop()
        """ Exit a with statement"""

    def start(self):
        self.steeringCtrl.startPwm()
        self.speedCtrl.startPwm()
        self.camera.startThread()
        # Start a fullscreen raw preview
        if self.conf["DISPLAY"]["show_cam_preview"]:
            self.camera.start_preview()

    def stop(self):
        self.camera.stopThread()
        self.camera.stop_preview()
        self.steeringCtrl.stopPwm()
        self.speedCtrl.stopPwm()
        
        
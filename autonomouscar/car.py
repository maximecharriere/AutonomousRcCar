from actuator_controller import SteeringController, SpeedController
from my_camera import PicameraController

class Car():
    def __init__(self, conf):
        self.SpeedCtrl = SpeedController(
            pin = conf["PIN"]["pwm_speed"], 
            minDutyCycle = conf["CAR"]["speed_pwm_dc_min"],
            maxDutyCycle = conf["CAR"]["speed_pwm_dc_max"], 
            hardware=True)
        self.SteeringCtrl = SteeringController(
            pin = conf["PIN"]["pwm_steering"], 
            minDutyCycle = conf["CAR"]["steering_pwm_dc_min"],
            maxDutyCycle = conf["CAR"]["steering_pwm_dc_max"], 
            hardware=True)
        self.Camera = PicameraController(
            cam_param_dict = [(arg, value) for (arg, value) in conf['CAMERA']['parameters'].items() if value != None]
        )
        self.speedLimit = conf["CAR"]["default_speed_limit"]

    def start(self):
        self.SteeringCtrl.startPwm()
        self.SpeedCtrl.startPwm()
        
        
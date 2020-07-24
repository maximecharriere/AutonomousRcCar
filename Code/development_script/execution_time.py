#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
# ----------------------------------- Infos -----------------------------------


import timeit

setup_code = '''
import sys
sys.path.insert(0,'/home/pi/Documents/Code')

import cv2
from PIL import Image
from road_follower import RoadFollower
from objects_detector import ObjectsDetector
from obstacle_detector import ObstacleDetector
from scipy import stats
import my_lib
import numpy as np
from car import Car

conf = my_lib.load_configuration('/home/pi/Documents/Code/conf.yaml')
car_state = {
            'stop_flags': {
                'no_road'     : False,
                'stop_sign'   : False,
                'red_light'   : False,
                'obstacle'    : False,
                'manual_stop' : False
            },
            'speed_limit'     : conf["CAR"]["real_speed_25"]
        }

car = Car(conf = conf)
roadFollower = RoadFollower(
    conf = conf,
    camera = car.camera, 
    steeringCtrl = car.steeringCtrl, 
    car_state = car_state)
objectDetector = ObjectsDetector(
    conf = conf, 
    camera  = car.camera, 
    car_state = car_state,
    max_fps = 2000)

# car.start()
img = cv2.imread("/home/pi/Documents/Code/resources/executionTimeTest_sample_x240.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
'''

code_calibration = '''
roadFollower.imgRectifier.undistort(img)
'''

code_warp = '''
roadFollower.imgWarper.warp(img)
'''

code_BgrToHsv = '''
cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
'''

code_threshold = '''
my_lib.inRangeHSV(
            src = img, 
            lowerb = conf["IMAGE_PROCESSING"]["hsv_threshold"]["low"], 
            upperb = conf["IMAGE_PROCESSING"]["hsv_threshold"]["high"])
'''

code_connectedComponents = '''
cv2.connectedComponentsWithStats(img, ltype=cv2.CV_16U)
'''

code_polyfit = '''
all_coef = []
std_deviation = []
for i in range(line_label.size):
    y, x = np.where(img_labeled == line_label[i])
    # Polyfit
    p, V, = np.polyfit(y, x, 1, cov = True) # inversion of x and y because lines are mostly vertical
    std_err = np.sqrt(V[0,0])
    all_coef.append(p)
    std_deviation.append(std_err)
'''

code_linregress = '''
all_coef = []
std_deviation = []
for i in range(line_label.size):
    y, x = np.where(img_labeled == line_label[i])
    # Linregress
    slope, intercept, r_value, p_value, std_err = stats.linregress(y, x)
    p = (slope, intercept)
    all_coef.append(p)
    std_deviation.append(std_err)
'''

code_obstacleDetector = '''
car.ultrasonicSensor.getDistance()
'''

code_objectDetector = '''
objectDetector.engine.detect_with_image(
                Image.fromarray(car.camera.current_frame), 
                keep_aspect_ratio =False, 
                relative_coord=False,
                threshold=conf['OBJECT_DETECTION']['match_threshold'],
                top_k=conf['OBJECT_DETECTION']['max_obj'])
'''

code_getSteering = '''
roadFollower._getSteering(img)
'''


print(timeit.timeit(stmt=code_getSteering, setup=setup_code, number=1000))

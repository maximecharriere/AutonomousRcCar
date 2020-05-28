import sys
sys.path.append('..')
from autonomouscar import camera_calibration, perspective_warp, my_lib
import numpy as np
import cv2
from scipy import stats

## Parameters
camResolution=(640, 480)
min_line_area = 0.5 #in  of img area
low_H = 0
low_S = 0
low_V = 0
high_H = 180
high_S = 255
high_V = 80
perspectiveWarpPoints = [(173, 1952),(2560, 1952),(870, 920),(1835, 920)]
perspectiveWarpPointsResolution = (2592, 1952)

frameBGR = cv2.imread("/home/pi/Documents/AutonomousRcCar/autonomouscar/resources/executionTimeTest_sample_x480.jpg",cv2.IMREAD_COLOR)
frameBGR_calibrate = camera_calibration.undistort(frameBGR, calParamFile="/home/pi/Documents/AutonomousRcCar/autonomouscar/resources/cameraCalibrationParam_V2.pickle",crop=True)
frameBGR_warped = perspective_warp.warp(frameBGR_calibrate, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)
frameHSV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HSV)
frameThreshold = cv2.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))
frameThreshold = cv2.erode(frameThreshold,kernel=np.ones((3,3)))
retval, labels_img, stats, centroids = cv2.connectedComponentsWithStats(frameThreshold, ltype=cv2.CV_16U)
line_label = np.where(stats[1:,cv2.CC_STAT_AREA] >= min_line_area)[0]+1

cv2.imshow("Polyfit", frameBGR)
cv2.imshow("Originsl", frameThreshold)
key = cv2.waitKey(0)
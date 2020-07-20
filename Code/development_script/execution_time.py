import timeit

setup_code = '''
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
_, labels_img, blobStats, _  = cv2.connectedComponentsWithStats(frameThreshold, ltype=cv2.CV_16U)
line_label = np.where(blobStats[1:,cv2.CC_STAT_AREA] >= min_line_area)[0]+1
'''

code_calibration = '''
frameBGR_calibrate = camera_calibration.undistort(frameBGR, calParamFile="/home/pi/Documents/AutonomousRcCar/autonomouscar/resources/cameraCalibrationParam_V2.pickle",crop=True)
'''

code_warp = '''
frameBGR_warped = perspective_warp.warp(frameBGR_calibrate, perspectiveWarpPoints, [80, 0, 80, 0], perspectiveWarpPointsResolution)'''

code_BgrToHsv = '''
frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
'''

code_threshold = '''
frameThreshold = cv2.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))
'''

code_erode = '''
frameThreshold = cv2.erode(frameThreshold,kernel=np.ones((3,3)))
'''

code_connectedComponents = '''
retval, labels_img, stats, centroids = cv2.connectedComponentsWithStats(frameThreshold, ltype=cv2.CV_16U)
'''

code_polyfit = '''
coef = []
stdDeviation = []
for i in range(line_label.size):
    y, x = np.where(labels_img == line_label[i])
    p, V, = np.polyfit(y, x, 1, cov = True) # inversion of x and y because lines are mostly vertical
    coef.append(p)
    stdDeviation.append(np.sqrt(np.diag(V)))
'''

code_linregress = '''
coef = []
stdDeviation = []
for i in range(line_label.size):
    y, x = np.where(labels_img == line_label[i])
    slope, intercept, r_value, p_value, std_err = stats.linregress(y, x) # inversion of x and y because lines are mostly vertical
    coef.append((slope, intercept))
    stdDeviation.append(std_err)
'''

code_full = '''
frameBGR_calibrate = cameraCalibration.undistort(img, calParamFile="/home/pi/Documents/AutonomousRcCar/Code/CameraCalibration/cameraCalibrationParam_V2.pickle",crop=True)
frameBGR_warped = perspectiveWarp.perspective_warp(frameBGR_calibrate, perspectiveWarpPoints, [30, 0, 30, 0], perspectiveWarpPointsResolution)
frameHSV = cv2.cvtColor(frameBGR_warped, cv2.COLOR_BGR2HSV)
frameThreshold = cv2.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))
frameThreshold = cv2.erode(frameThreshold,kernel=np.ones((3,3))) #to minimise the number of components and speed processing time (à mesurer)
retval, labels_img, stats, centroids = cv2.connectedComponentsWithStats(frameThreshold, ltype=cv2.CV_16U) #https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html
line_label = np.where(stats[1:,cv2.CC_STAT_AREA] >= min_line_area*frameThreshold.size/100)[0]+1 #the "1" is to exclude label 0 who is the background 
coef = []
for i in range(line_label.size):
    y,x  = np.where(labels_img==line_label[i])
    coef.append(np.polyfit(y, x, 1)) #inversion of x and y because lines are mostly vertical
coefNp = np.array(coef)
slop = np.mean(coefNp[:,0])
'''


print(timeit.timeit(stmt=code_linregress, setup=setup_code, number=1000))

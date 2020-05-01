import numpy as np
import cv2
import matplotlib.pyplot as plt

img = cv2.imread('/home/pi/Documents/AutonomousRcCar/ARC_Code/CameraCalibration/CalibrationResult_V2_roi.png')
img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
plt.imshow(img,cmap = 'gray')
plt.show()
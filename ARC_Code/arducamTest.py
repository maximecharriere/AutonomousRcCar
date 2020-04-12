import cv2 as cv

## Parameters
low_H = 24
low_S = 23
low_V = 195
high_H = 37
high_S = 255
high_V = 255
window_capture_name = 'Video Capture'

frameBGR = cv.imread('/home/pi/Documents/AutonomousRcCar/Images/picam.jpg')
frameBGR = cv.rotate(frameBGR, cv.ROTATE_180)
frameHSV = cv.cvtColor(frameBGR, cv.COLOR_BGR2HSV)

frameThreshold = cv.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))
print(f"Original: {frameBGR.shape}")
print(f"HSV: {frameHSV.shape}")
print(f"Threshold: {frameThreshold.shape}")
print(frameThreshold.min())
print(frameThreshold.max())

cv.namedWindow(window_capture_name)
cv.imshow(window_capture_name, frameBGR)
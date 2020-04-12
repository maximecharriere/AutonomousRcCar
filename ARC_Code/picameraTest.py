import time
import picamera
import numpy as np
import cv2
from datetime import datetime

## Parameters
low_H = 24
low_S = 23
low_V = 195
high_H = 37
high_S = 255
high_V = 255
window_capture_name = 'Video Capture'
SavePicture = True
ShowPreview = False

with picamera.PiCamera(resolution=(2592, 1920), framerate=30, sensor_mode=2) as camera:
    ## Camera configuration
    camera.exposure_mode = 'auto' 
    camera.meter_mode = 'average'
    camera.contrast = 70
    camera.drc_strength = 'high'
    camera.saturation = 0
    camera.sharpness = 100
    # camera.shutter_speed = 62974
    # camera.ISO = 100
    # camera.brightness = 50
    # camera.awb_mode  = 'off'
    # camera.awb_gains = (256/256,256/256)
    time.sleep(2)    

    ## Preview
    if ShowPreview:
        camera.start_preview()

    ## Save the picture
    if SavePicture:
        camera.capture(f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_cts-{camera.contrast}_DRC-{camera.drc_strength}_sat-{camera.saturation}_sharp-{camera.sharpness}_awbr-{float(camera.awb_gains[0]):.1f}_awbb-{float(camera.awb_gains[1]):.1f}_expMode-{camera.exposure_mode}_expSpeed-{camera.exposure_speed}.jpg")
    
    ## OpenCV computation
    frameBGR = np.empty((1920, 2592, 3), dtype=np.uint8)
    camera.capture(frameBGR, 'bgr')
    frameHSV = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
    frameThreshold = cv2.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))

    ## Show image
    cv2.namedWindow(window_capture_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_capture_name, frameBGR)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    ## Close properly
    camera.stop_preview()
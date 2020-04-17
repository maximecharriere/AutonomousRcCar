import time
import picamera
import numpy as np
import cv2
from datetime import datetime
from matplotlib import pyplot as plt

## Parameters
low_H = 0
low_S = 157
low_V = 254
high_H = 22
high_S = 255
high_V = 255
window_capture_name = 'Video Capture'
SavePicture = False
ShowPreview = False
OpenCvComputation = True

with picamera.PiCamera(resolution=(2592, 1920), framerate=30, sensor_mode=2) as camera:
    ## Camera configuration
    camera.exposure_mode = 'auto' 
    camera.meter_mode = 'average'
    camera.contrast = 100
    camera.drc_strength = 'high'
    camera.saturation = 100
    camera.sharpness = 100
    # camera.shutter_speed = 62974
    # camera.ISO = 100
    # camera.brightness = 50
    # camera.awb_mode  = 'off'
    #camera.awb_gains = (350/256,400/256)
    time.sleep(2)  

    ## Preview
    if ShowPreview:
        camera.start_preview()

    ## Save the picture
    if SavePicture:
        camera.capture(f"Images/ConfigCamera/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_cts-{camera.contrast}_DRC-{camera.drc_strength}_sat-{camera.saturation}_sharp-{camera.sharpness}_awbr-{float(camera.awb_gains[0]):.1f}_awbb-{float(camera.awb_gains[1]):.1f}_expMode-{camera.exposure_mode}_expSpeed-{camera.exposure_speed}.jpg")
    
    ## OpenCV computation
    if OpenCvComputation:
        frameBGR = np.empty((1920, 2592, 3), dtype=np.uint8)
        camera.capture(frameBGR, 'bgr')
        frameHSV = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
        frameThreshold = cv2.inRange(frameHSV,  (low_H, low_S, low_V), (high_H, high_S, high_V))
        frameEdge = cv2.Canny(frameThreshold, 100,400)

        plt.subplot(121),plt.imshow(frameThreshold,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(frameEdge,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
        plt.show()

        # ## Show image
        # cv2.namedWindow(window_capture_name, cv2.WINDOW_NORMAL)
        # cv2.imshow(window_capture_name, frameThreshold)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    ## Close properly
    camera.stop_preview()
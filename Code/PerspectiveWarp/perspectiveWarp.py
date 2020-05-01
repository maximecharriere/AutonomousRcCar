import numpy as np
import cv2
import matplotlib.pyplot as plt

def perspective_warp(img, dst_size=(2592, 1944)):

    src=np.float32([(1120, 760),(1600, 760),(313, 1444),(2435, 1444)])
    dst=np.float32([(0.1,0.1), (0.9, 0.1), (0.1,0.9), (0.9,0.9)])
    dst = dst * np.float32(dst_size)
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped


img = cv2.imread('/home/pi/Documents/AutonomousRcCar/ARC_Code/CameraCalibration/CalibrationResult_V2_roi.png')
img_warped = perspective_warp(img)
cv2.imwrite("/home/pi/Documents/AutonomousRcCar/ARC_Code/PerspectiveWarp/warpedV2.png", img_warped) 


# plt.subplot(121),plt.imshow(img,cmap = 'gray')
# plt.title('Original Image')
# plt.subplot(122),plt.imshow(img_warped,cmap = 'gray')
# plt.title('Warped Image')
# plt.show()

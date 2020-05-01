import numpy as np
import cv2
import glob
import pickle
import matplotlib.pyplot as plt



def getCameraCalibrationParameters(nRow, nCol, imgDirectory, calParamFile=None, saveDrawedImg=False):
    # prepare real world point as [x,y,z] coordinate. Z always on the same plane, so it keep 0. 
    # XY are (0,0),(1,0),(2,0),...,(8,0), (0,1),(1,1),...,(8,1), (0,2),......,(8,5)
    objp = np.zeros((nRow*nCol,3), np.float32)
    objp[:,:2] = np.mgrid[0:nCol,0:nRow].T.reshape(-1,2)

    # Arrays to store object points (real world) and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # Calibration images folder
    images = glob.glob(imgDirectory)

    # Find chestboard corners in all images
    for i, fname in enumerate(images):
        # Load image
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (nCol,nRow),None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            # Refines the corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Max 30 iter and epsilon of 0.001
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and save the corners in new image
            if saveDrawedImg:
                img = cv2.drawChessboardCorners(img, (nCol,nRow), corners2,ret)
                cv2.imwrite(fname.replace(".jpg","_findedCorners.jpg"), img) 
        else:
            print(f"Corner not found in img {fname}")
        # print progression
        print(f"Image {i+1}/{len(images)} analysed")
    
    # Calibrate the camera
    img_size = (img.shape[1], img.shape[0])
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None,None)

    # Save camera calibration for later use
    if calParamFile is not None:
        calParamPickle = {}
        calParamPickle['mtx'] = mtx
        calParamPickle['dist'] = dist
        pickle.dump(calParamPickle, open(calParamFile, 'wb') )

    return mtx, dist

def undistort(img, calParamFile):
    try:
        with open(calParamFile, mode='rb') as fd:
            file = pickle.load(fd)    
            mtx = file['mtx']
            dist = file['dist']
            h,  w = img.shape[:2]
            mtx_new, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(img.shape[1], img.shape[0]), 1)
            undistortedImg = cv2.undistort(img, mtx, dist, None, None) # (img, mtx, dist, None, mtx_new) to get full image
            return undistortedImg
    except FileNotFoundError :
        print("File with calibration parameters not found")



#getCameraCalibrationParameters(nRow=5, nCol=8, imgDirectory='/home/pi/Documents/AutonomousRcCar/ARC_Code/CameraCalibration/ImagesV2andV3/*.jpg', calParamFile="/home/pi/Documents/AutonomousRcCar/ARC_Code/CameraCalibration/cameraCalibrationParam_V2andV3.pickle", saveDrawedImg=True)

img = cv2.imread('/home/pi/Documents/AutonomousRcCar/Images/RoadV3.jpg')
# img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

img_undistort = undistort(img, calParamFile="/home/pi/Documents/AutonomousRcCar/ARC_Code/CameraCalibration/cameraCalibrationParam_V2.pickle")
cv2.imwrite("/home/pi/Documents/AutonomousRcCar/ARC_Code/CameraCalibration/CalibrationResult_V2.png", img_undistort) 


# plt.subplot(121),plt.imshow(img,cmap = 'gray')
# plt.title('Original Image')
# plt.subplot(122),plt.imshow(img_undistort,cmap = 'gray')
# plt.title('Undistort Image')
# plt.show()

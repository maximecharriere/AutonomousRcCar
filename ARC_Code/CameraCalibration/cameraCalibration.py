import numpy as np
import cv2
import glob
import pickle
import matplotlib.pyplot as plt

def getCameraCalibrationParameters(nRow=6, nCol=9, imgDirectory='Images/*.jpg', calParamFile=None, saveDrawedImg=False):
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
    for fname in images:
        # Load image
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (nCol,nRow),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            # termination criteria
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) #Max 30 iter and epsilon of 0.001
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and save the corners
            if saveDrawedImg:
                img = cv2.drawChessboardCorners(img, (nCol,nRow), corners2,ret)
                cv2.imwrite(fname.replace(".jpg","_findedCorners.jpg"), img) 
        else:
            print(f"Corner not found in img {fname}")

    
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
            undistortedImg = cv2.undistort(img, mtx, dist, None, mtx_new)
            return undistortedImg
    except FileNotFoundError :
        print("File with calibration parameters not found")



getCameraCalibrationParameters(nRow=6, nCol=9, imgDirectory='Images/*.jpg', calParamFile="cameraCalibrationParam.pickle", saveDrawedImg=False)

img = cv2.imread('Images/5.jpg')
img_undistort = undistort(img, calParamFile="cameraCalibrationParam.pickle")

plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image')
plt.subplot(122),plt.imshow(img_undistort,cmap = 'gray')
plt.title('Edge Image')
plt.show()

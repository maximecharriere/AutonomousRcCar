import numpy as np
import cv2
import matplotlib.pyplot as plt


def order_points(pts): 
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros_like(pts)

    # Get the index of the 4 points ordered along y axis (axis nbr 1)
    verticalIndex = pts[:,1].argsort()
    
    ## Top-left point:
    # Take the two points on top of the image
    topPoints = pts[verticalIndex][:2]
    # Find the min value along the x axis (axis nbr 0)
    pointIdx = topPoints[:,0].argmin()
    rect[0] = topPoints[pointIdx]

    ## Top-right point:
    # Find the max value along the x axis (axis nbr 0)
    pointIdx = topPoints[:,0].argmax()
    rect[1] = topPoints[pointIdx]

    # Bottom-right point:
    # Take the two points on bottom of the image
    topPoints = pts[verticalIndex][2:]
    # Find the max value along the x axis (axis nbr 0)
    pointIdx = topPoints[:,0].argmax()
    rect[2] = topPoints[pointIdx]

    ## Top-right point:
    # Find the min value along the x axis (axis nbr 0)
    pointIdx = topPoints[:,0].argmin()
    rect[3] = topPoints[pointIdx]

    return rect


def perspective_warp(img, imgPoints, margin_pc=[0,0,0,0], refImageResolution = None): #Inspired from https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/
    # convert the imgPoints and margin list into a numpy array
    imgPoints = np.array(imgPoints,dtype="float32")
    margin_pc = np.array(margin_pc)

    # repositioning the points according to image shape if the base resolution of the points is given
    if refImageResolution is not None:
        imgPoints[:,0] *= (img.shape[1] / refImageResolution[0])
        imgPoints[:,1] *= (img.shape[0] / refImageResolution[1])

    # Convert margin from % to pxs
    margin_pxs = np.zeros(4, dtype=int)
    margin_pxs[::2] = (margin_pc[::2]*img.shape[1] / 100.0).astype(int)
    margin_pxs[1::2] = (margin_pc[1::2]*img.shape[0] / 100.0).astype(int)

    # obtain a consistent order of the points and unpack them individually
    rect= order_points(imgPoints)
    (tl, tr, br, bl) = rect

    ##          Compute the size of the new image
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]]) + margin_pxs[:2]
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst.astype("float32"))
    return cv2.warpPerspective(img, M, (maxWidth+sum(margin_pxs[::2]), maxHeight+sum(margin_pxs[1::2])),borderValue =(255,255,255))
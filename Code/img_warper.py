import numpy as np
import cv2

class ImgWarper():
    def __init__(self, imgShape, corners, realWorldCornersDistance, margin_pc=[0,0,0,0], cornersImageResolution = None): #Inspired from https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/       
        # convert the corners and margin list into a numpy array
        corners = np.array(corners,dtype="float32")
        margin_pc = np.array(margin_pc)

        assert len(corners) == 4, "4 corners have to be given"

        # repositioning the corners according to image shape if the base resolution of the corners is given
        if cornersImageResolution is not None:
            corners[:,0] *= (imgShape[1] / cornersImageResolution[0])
            corners[:,1] *= (imgShape[0] / cornersImageResolution[1])

        # Convert margin from % to pxs
        self.margin_pxs = np.zeros(4, dtype=int)
        self.margin_pxs[::2] = (margin_pc[::2]*imgShape[1] / 100.0).astype(int)
        self.margin_pxs[1::2] = (margin_pc[1::2]*imgShape[0] / 100.0).astype(int)

        # obtain a consistent order of the points and unpack them individually
        corners= _orderCorners(corners)
        (tl, tr, br, bl) = corners

        ## Compute the ratio between the hight and width of the
        #   points in real world
        wh_ratio = realWorldCornersDistance[0]/realWorldCornersDistance[1]

        ##          Compute the size of the new image
        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        self.maxWidth = max(int(widthA), int(widthB))
        self.maxHeight = int(self.maxWidth*wh_ratio)

        # Construct the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [self.maxWidth, 0],
            [self.maxWidth, self.maxHeight],
            [0, self.maxHeight]]) + self.margin_pxs[:2]
        # compute the perspective transform matrix and then apply it
        self.M = cv2.getPerspectiveTransform(corners, dst.astype("float32"))

    def warp(self, img):
        return cv2.warpPerspective(img, self.M, (self.maxWidth+sum(self.margin_pxs[::2]), self.maxHeight+sum(self.margin_pxs[1::2])))




def _orderCorners(pts): 
    '''initialzie a list of coordinates that will be ordered
    such that the first entry in the list is the top-left,
    the second entry is the top-right, the third is the
    bottom-right, and the fourth is the bottom-left
    '''
    
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
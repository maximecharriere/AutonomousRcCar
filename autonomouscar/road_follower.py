import my_lib
import cv2

class RoadFollower():
    def __init__(self, imgShape, conf):
        self.conf = conf
        self.imgWarper = ImgWarper(
            imgShape = imgShape, 
            imgPoints = self.conf["IMAGE_PROCESSING"]["perspective_warp"]["points"], 
            realWorldPointsDistance = self.conf["IMAGE_PROCESSING"]["perspective_warp"]["realworld_line_distance"], 
            margin_pc = self.conf["IMAGE_PROCESSING"]["perspective_warp"]["warp_margin"], 
            refImageResolution = self.conf["IMAGE_PROCESSING"]["perspective_warp"]["points_resolution"])
        self.slop_history = {
            "lastValue": 0.0, 
            "lastUpdate" : self.conf["IMAGE_PROCESSING"]["line_filtering"]["history_size"]+1
        }

    
    def getSteering(self, img):
        # Transform the image to see it from above
        img_warped = self.imgWarper.warp(img)
        # Apply a threshold on the HSV values of the image
        img_HSV = cv2.cvtColor(img_warped, cv2.COLOR_RGB2HSV)
        img_thresholded = my_lib.inRangeHSV(
            src = img_HSV, 
            lowerb = self.conf["IMAGE_PROCESSING"]["hsv_threshold"]["low"], 
            upperb = self.conf["IMAGE_PROCESSING"]["hsv_threshold"]["high"])
        # Label connected pixels to form groups
        # https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html
        _, img_labeled, blobs_stats, _ = cv2.connectedComponentsWithStats(
            img_thresholded, ltype=cv2.CV_16U)
        # Take only big connected pixel groups
        line_label = np.where(
            blobs_stats[1:, cv2.CC_STAT_AREA] >= self.conf["IMAGE_PROCESSING"]["line_filtering"]["min_area"]*img_thresholded.size/100)[0]+1 # the "1" is to exclude label 0 who is the background
        # Quit if no lines found
        if (line_label.size == 0):
            self.slop_history["lastUpdate"]+=1
            print(f"No line found ({self.slop_history['lastUpdate']})")
            return
        # Polyfit
        all_coef = []
        std_deviation = []
        for i in range(line_label.size):
            y, x = np.where(labels_img == line_label[i])
            p, V, = np.polyfit(y, x, 1, cov = True) # inversion of x and y because lines are mostly vertical
            all_coef.append(p)
            std_deviation.append(np.sqrt(V[0,0]))
        # Conversion into np array
        all_coef = np.array(all_coef)
        std_deviation = np.array(std_deviation)

        ## Selection of correct lines
        # Lines with a small standard deviation.
        wantedLines_mask = (std_deviation < self.conf["IMAGE_PROCESSING"]["line_filtering"]["max_SD"])
        # Line with relatively the same slope as before
        # If the last update of the history is old, this criterion is not taken into account
        if(self.slop_history["lastUpdate"] < self.conf["IMAGE_PROCESSING"]["line_filtering"]["history_size"]):
            wantedLines_mask &= (
                (all_coef[:,0]<self.slop_history["lastValue"]+self.conf["IMAGE_PROCESSING"]["line_filtering"]["slop_margin"]) & 
                (all_coef[:,0]>self.slop_history["lastValue"]-self.conf["IMAGE_PROCESSING"]["line_filtering"]["slop_margin"])
            )
        # Quit if no correct lines found
        if (np.where(wantedLines_mask)[0].size == 0):
            self.slop_history["lastUpdate"]+=1
            print(f"No correct line found ({self.slop_history['lastUpdate']})")
            return

        # Classify left/right lines
        bottom = img_thresholded.shape[0]
        lines_bottom_intercept = np.zeros(line_label.size)
        lines_bottom_intercept = all_oef[:, 0]*bottom + allCoef[:,1]
        leftLines_mask = wantedLines_mask & (lines_bottom_intercept <= img_thresholded.shape[1]/2)
        rightLines_mask = wantedLines_mask & (lines_bottom_intercept > img_thresholded.shape[1]/2)

        ## Compute the mean slop of all lines
        slop = np.mean(all_coef[wantedLines_mask,0])
        # Update history
        self.slop_history["lastUpdate"] = 0
        self.slop_history["lastValue"] = slop

        ## Compute the off-centre value of the car
        leftLine_bottom_intercept = lines_bottom_intercept[leftLines_mask].mean()
        rightLine_bottom_intercept = lines_bottom_intercept[rightLines_mask].mean()

        if  my_lib.isaN(leftLine_bottom_intercept) and my_lib.isaN(rightLine_bottom_intercept): #both line found
            road_center = np.mean((leftLine_bottom_intercept, rightLine_bottom_intercept))
        elif my_lib.isaN(leftLine_bottom_intercept): #left line found
            road_center = leftLine_bottom_intercept + self.conf["IMAGE_PROCESSING"]["line_spacing"]/2
        elif my_lib.isaN(rightLine_bottom_intercept): #right line found
            road_center = rightLine_bottom_intercept - self.conf["IMAGE_PROCESSING"]["line_spacing"]/2
        else: #no line found
            raise Exception("No line found. It should not be possible, at this stage of the code, not to find a line")

        off_centre = img_thresholded.shape[1]/2 - road_center
        off_centre_norm_clamped = my_lib.clamp(off_centre/(self.conf["IMAGE_PROCESSING"]["line_spacing"]/2),-1,1) #Normalize centerDiff that can be between +-lineSpacing/2 to become between +-1
        # Applies to the car's off-centre value (which is linear) a "tan" function to accentuate the value as the car moves away from the centre.
        off_centre_tan = np.tan(off_centre_norm_clamped*np.pi/4)


        ## Combine angle and off-center of the car to compute steering
        slop_clamped = my_lib.clamp(slop, -1, 1)
        car_steering_norm = my_lib.mix(slop_clamped, centerDiff_tan, np.abs(slop_clamped))

        return car_steering_norm
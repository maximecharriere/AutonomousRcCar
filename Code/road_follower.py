import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

import my_lib
import cv2
import numpy as np
import time 
import math
from scipy import stats
from threading import Thread
from img_warper import ImgWarper
from img_rectifier import ImgRectifier

class RoadFollower():
    # Var to stop the thread
    stopped = False
    drawed_img = None

    def __init__(self, conf, camera, steeringCtrl, car_state, current_threads_fps=None):
        self.camera = camera
        self.steeringCtrl = steeringCtrl
        self.conf = conf
        self.car_state = car_state
        self.current_threads_fps = current_threads_fps
        self.imgRectifier = ImgRectifier(
            imgShape = self.conf["ROAD_FOLLOWING"]["img_resolution"][::-1],
            calParamFile = os.path.join(currentdir, self.conf["ROAD_FOLLOWING"]["calibration"]["param_file"]))
        self.imgWarper = ImgWarper(
            imgShape = self.conf["ROAD_FOLLOWING"]["img_resolution"][::-1], 
            corners = self.conf["ROAD_FOLLOWING"]["perspective_warp"]["points"], 
            realWorldCornersDistance = self.conf["ROAD_FOLLOWING"]["perspective_warp"]["realworld_line_distance"], 
            margin_pc = self.conf["ROAD_FOLLOWING"]["perspective_warp"]["warp_margin"], 
            cornersImageResolution = self.conf["ROAD_FOLLOWING"]["perspective_warp"]["points_resolution"])
        self.slop_history = {
            "lastValue": 0.0, 
            "lastUpdate" : self.conf["ROAD_FOLLOWING"]["line_filtering"]["history_size"]+1
        }


    def __enter__(self):
        """ Entering a with statement """
        self.startThread()
        return self
    
    def __exit__(self, exception_type, exception_value, traceback):
        self.stopThread()
        """ Exit a with statement"""

    def startThread(self):
        # start the thread to follow the road
        t = Thread(target=self._run, name=self.__class__.__name__, args=())
        t.start()
        return self

    def stopThread(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def _run(self):
        self.car_state['stop_flags']['no_road'] = True
        start_time = time.time()
        while not self.stopped:
            img  = cv2.resize(src=self.camera.current_frame ,dsize=tuple(self.conf["ROAD_FOLLOWING"]["img_resolution"])) #self.camera.current_frame #
            steering_value = self._getSteering(img, draw_result= self.conf["DISPLAY"]["show_plots"])
            if (steering_value): self.steeringCtrl.angle(steering_value)

            # Check if no lines is found from a long time and stop car
            self.car_state['stop_flags']['no_road'] = (self.slop_history['lastUpdate'] > self.conf["ROAD_FOLLOWING"]["line_filtering"]["history_size"])

            if self.current_threads_fps is not None:
                self.current_threads_fps[self.__class__.__name__] = 1/(time.time()-start_time)
                start_time = time.time()


    def _getSteering(self, img, draw_result = False):
        '''
        RETURN
        ------
        car_steering_norm : double
            Normalized value between -1 and 1 determining direction and value in which the car must turn
            If None, no line is found in img

        '''
        

        car_steering_norm = None

        # Transform the image to see it from above
        
        img = self.imgRectifier.undistort(img)
        img_warped = self.imgWarper.warp(img)
        

        # Apply a threshold on the HSV values of the image
        img_HSV = cv2.cvtColor(img_warped, cv2.COLOR_RGB2HSV)
        
        img_thresholded = my_lib.inRangeHSV(
            src = img_HSV, 
            lowerb = self.conf["ROAD_FOLLOWING"]["hsv_threshold"]["low"], 
            upperb = self.conf["ROAD_FOLLOWING"]["hsv_threshold"]["high"])
        
        # img_thresholded = cv2.morphologyEx(img_thresholded, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))
        
        # Label connected pixels to form groups
        # https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html
        _, img_labeled, blobs_stats, _ = cv2.connectedComponentsWithStats(
            img_thresholded, ltype=cv2.CV_16U)
        
        # Take only big connected pixel groups
        
        line_label = np.where(
            blobs_stats[1:, cv2.CC_STAT_AREA] >= self.conf["ROAD_FOLLOWING"]["line_filtering"]["min_area"]*img_thresholded.size/100)[0]+1 # the "1" is to exclude label 0 who is the background
        # Quit if no lines found
        
        if (line_label.size == 0):
            self.slop_history["lastUpdate"]+=1
            print(f"No line found ({self.slop_history['lastUpdate']})")
        else:
            # Find conected pixels shape
            all_coef = []
            std_deviation = []
            
            for i in range(line_label.size):
                y, x = np.where(img_labeled == line_label[i])
                # Polyfit
                p, V, = np.polyfit(y, x, 1, cov = True) # inversion of x and y because lines are mostly vertical
                std_err = np.sqrt(V[0,0])
                all_coef.append(p)
                std_deviation.append(std_err)

            
            # Conversion into np array
            all_coef = np.array(all_coef)
            std_deviation = np.array(std_deviation)

            ## Selection of correct lines
            # Lines with a small standard deviation.
            wantedLines_mask = (std_deviation < self.conf["ROAD_FOLLOWING"]["line_filtering"]["max_SD"])
            # Line with relatively the same slope as before
            # If the last update of the history is old, this criterion is not taken into account
            if(self.slop_history["lastUpdate"] < self.conf["ROAD_FOLLOWING"]["line_filtering"]["history_size"]):
                wantedLines_mask &= (
                    (all_coef[:,0]<self.slop_history["lastValue"]+self.conf["ROAD_FOLLOWING"]["line_filtering"]["slop_margin"]) & 
                    (all_coef[:,0]>self.slop_history["lastValue"]-self.conf["ROAD_FOLLOWING"]["line_filtering"]["slop_margin"])
                )

            # Classify left/right lines
            bottom = img_thresholded.shape[0]
            lines_bottom_intercept = np.zeros(line_label.size)
            lines_bottom_intercept = all_coef[:, 0]*bottom + all_coef[:,1]
            leftLines_mask = wantedLines_mask & (lines_bottom_intercept <= img_thresholded.shape[1]/2)
            rightLines_mask = wantedLines_mask & (lines_bottom_intercept > img_thresholded.shape[1]/2)

            # Quit if no correct lines found
            if (np.where(wantedLines_mask)[0].size == 0):
                self.slop_history["lastUpdate"]+=1
                print(f"No correct line found ({self.slop_history['lastUpdate']})")
            else:
                ## Compute the mean slop of all lines
                slop = np.mean(all_coef[wantedLines_mask,0])
                # Update history
                self.slop_history["lastUpdate"] = 0
                self.slop_history["lastValue"] = slop

                ## Compute the off-centre value of the car
                leftLine_bottom_intercept = lines_bottom_intercept[leftLines_mask].mean()
                rightLine_bottom_intercept = lines_bottom_intercept[rightLines_mask].mean()
                line_spacing = self.conf["ROAD_FOLLOWING"]["line_spacing"]/100 * self.conf["ROAD_FOLLOWING"]["img_resolution"][0]
                if  my_lib.isaN(leftLine_bottom_intercept) and my_lib.isaN(rightLine_bottom_intercept): #both line found
                    road_center = np.mean((leftLine_bottom_intercept, rightLine_bottom_intercept))
                elif my_lib.isaN(leftLine_bottom_intercept): #left line found
                    road_center = leftLine_bottom_intercept + line_spacing/2
                elif my_lib.isaN(rightLine_bottom_intercept): #right line found
                    road_center = rightLine_bottom_intercept - line_spacing/2
                else: #no line found
                    raise Exception("No line found. It should not be possible, at this stage of the code, not to find a line")

                off_centre = img_thresholded.shape[1]/2 - road_center
                off_centre_norm_clamped = my_lib.clamp(off_centre/(line_spacing/2),-1,1) #Normalize centerDiff that can be between +-lineSpacing/2 to become between +-1
                # Applies to the car's off-centre value (which is linear) a "tan" function to accentuate the value as the car moves away from the centre.
                off_centre_tan = np.tan(off_centre_norm_clamped*np.pi/4)


                ## Combine angle and off-center of the car to compute steering
                slop_clamped = my_lib.clamp(slop, -1, 1)
                car_steering_norm  =my_lib.clamp(my_lib.mix(slop_clamped, off_centre_tan, np.abs(slop_clamped))*1.5, -1, 1) 
            
            
        # Draw a img with line and info if required
        if draw_result:
            # Generate image with different gray level for each connected pixels groups
            img_labelized = np.zeros_like(img_thresholded)
            if (line_label.size > 0):
                color_step = int(255/line_label.size)
                for i in range(line_label.size):
                    img_labelized[np.where(img_labeled == line_label[i])] = color_step*(i+1)

            # Creat a color img
            drawed_result = np.dstack((img_labelized, img_labelized, img_labelized))
            # Draw polyfit
            draw_y = np.linspace(0, img_thresholded.shape[0]-1, img_thresholded.shape[0], dtype=int)
            for i in range(line_label.size): 
                # Compute points
                draw_x = np.polyval(all_coef[i,:], draw_y)
                draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32)
                # Draw line
                if rightLines_mask[i]:
                    line_color = self.conf["DISPLAY"]["linecolor_right"]
                elif leftLines_mask[i]:
                    line_color = self.conf["DISPLAY"]["lineColor_left"]
                else:
                    line_color = self.conf["DISPLAY"]["lineColor_rejected"]
                cv2.polylines(drawed_result, [draw_points], False, line_color, math.ceil(self.conf["ROAD_FOLLOWING"]["img_resolution"][0]/150))
                # Draw text
                text_org = (
                    blobs_stats[line_label[i],cv2.CC_STAT_LEFT] + int(blobs_stats[line_label[i],cv2.CC_STAT_WIDTH]/2), 
                    blobs_stats[line_label[i],cv2.CC_STAT_TOP] + int(blobs_stats[line_label[i],cv2.CC_STAT_HEIGHT]/2)
                )
                drawed_result = cv2.putText(drawed_result,f"SD = {std_deviation[i]:.4f}",text_org, cv2.FONT_HERSHEY_SIMPLEX, self.conf["ROAD_FOLLOWING"]["img_resolution"][0]/400, self.conf["DISPLAY"]["textColor"], 1)
            self.drawed_img = drawed_result
        return car_steering_norm
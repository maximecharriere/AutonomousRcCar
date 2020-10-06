import sys, getopt, os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

import cv2
import numpy as np
import time
from edgetpu.detection.engine import DetectionEngine
from PIL import Image
from traffic_signs import *
from threading import Thread



class ObjectsDetector:
    """
    """
    # Var to stop the thread
    stopped = False
    drawed_img = None
    def __init__(self, conf, camera, car_state, max_fps, current_threads_fps=None):
        self.camera = camera
        self.car_state = car_state
        self.conf = conf
        self.min_execution_time = 1/max_fps
        self.current_threads_fps = current_threads_fps
        
        # Initialize engine.
        self.engine = DetectionEngine(os.path.join(currentdir, conf['OBJECT_DETECTION']['model_fname']))


        # Init a dictonary with obj_id and obj_label as key, and the corresponding traffic object class as value
        self.traffic_objects = dict.fromkeys([0, 'Battery'], Battery(conf)) 
        self.traffic_objects.update(dict.fromkeys([1, 'SpeedLimit25'], SpeedLimit(conf, conf['CAR']['real_speed_25'])))
        self.traffic_objects.update(dict.fromkeys([2, 'SpeedLimit50'], SpeedLimit(conf, conf['CAR']['real_speed_50'])))
        self.traffic_objects.update(dict.fromkeys([3, 'StopSign'], StopSign(conf)))
        self.traffic_objects.update(dict.fromkeys([4, 'TrafficLightGreen'], TrafficLight(conf, 'Green')))
        self.traffic_objects.update(dict.fromkeys([5, 'TrafficLightOff'], TrafficLight(conf, 'Off')))
        self.traffic_objects.update(dict.fromkeys([6, 'TrafficLightRed'], TrafficLight(conf, 'Red')))


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
        start_time = time.time()
        while not self.stopped:
            self.camera.new_frame_event.wait()
            # Run inference.
            objs = self.engine.detect_with_image(
                Image.fromarray(self.camera.current_frame), 
                keep_aspect_ratio =False, 
                relative_coord=False,
                threshold=self.conf['OBJECT_DETECTION']['match_threshold'],
                top_k=self.conf['OBJECT_DETECTION']['max_obj'])

            if self.conf["DISPLAY"]["show_plots"]:
                img = np.array(self.camera.current_frame, copy=True)

            for obj in objs:
                traffic_obj = self.traffic_objects[obj.label_id]
                obj_is_nearby = traffic_obj.is_nearby(obj)
                if obj_is_nearby:
                    traffic_obj.present = True

                # Print and draw detected objects.
                if self.conf["DISPLAY"]["show_plots"]:
                    cv2.rectangle(img,tuple(obj.bounding_box[0].astype(int)),tuple(obj.bounding_box[1].astype(int)), color=(0,255,0) if obj_is_nearby else (255,0,0))
                    cv2.putText(img, f"{traffic_obj.label} ({obj.score*100:.0f}%)",tuple(obj.bounding_box[0].astype(int)-(70,0)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.conf["DISPLAY"]["textColor"], 1)

            if self.conf["DISPLAY"]["show_plots"]:
                self.drawed_img = img

            # The 25 speed limit sign has priority.
            if self.traffic_objects['SpeedLimit25'].present:
                self.traffic_objects['SpeedLimit50'].present = False

            # The red light has priority.
            if self.traffic_objects['TrafficLightRed'].present:
                self.traffic_objects['TrafficLightOff'].present = False
                self.traffic_objects['TrafficLightGreen'].present = False

            # Each TrafficSignProcessor change the car state
            for traffic_object in (set(self.traffic_objects.values())):
                traffic_object.set_car_state(self.car_state)
                traffic_object.present = False

            elapsed_time = time.time() - start_time
            if (elapsed_time < self.min_execution_time):
                time.sleep(self.min_execution_time - elapsed_time)
            self.current_threads_fps[self.__class__.__name__] = 1/(time.time()-start_time)
            start_time = time.time()
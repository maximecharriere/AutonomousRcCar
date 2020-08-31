import time
from threading import Thread
from ultrasonic_sensor import NoEcho

class ObstacleDetector:
    # Var to stop the thread
    stopped = False
    def __init__(self, min_distance, distance_sensor, car_state, max_fps, current_threads_fps):
        self.sensor = distance_sensor
        self.car_state = car_state
        self.min_distance = min_distance
        self.min_execution_time = 1/max_fps
        self.current_threads_fps = current_threads_fps
    
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
        detection_count = 0
        MAX_DETECTION_COUNT = 3
        self.car_state['stop_flags']['obstacle'] = True
        start_time = time.time()
        while not self.stopped:
            try:
                distance = self.sensor.getDistance()
            except NoEcho:
                distance  = 9999
            #The sensor sometimes shows the wrong distance, 
            # which may cause the car to stop when there are no obstacles.
            # So during at least N measurements an obstacle must be detected to stop the car.
            if distance < self.min_distance/1000:
                detection_count +=1
            else :
                detection_count = 0

            self.car_state['stop_flags']['obstacle'] = detection_count >= MAX_DETECTION_COUNT
            
            elapsed_time = time.time() - start_time
            if (elapsed_time < self.min_execution_time):
                time.sleep(self.min_execution_time - elapsed_time)
            self.current_threads_fps[self.__class__.__name__] = 1/(time.time()-start_time)
            start_time = time.time()


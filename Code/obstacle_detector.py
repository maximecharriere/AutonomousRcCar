import time
from threading import Thread

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
        self.car_state['stop_flags']['obstacle'] = True
        # start_time = time.time()
        while not self.stopped:
            distance = self.sensor.getDistance()
            print(distance)
            self.car_state['stop_flags']['obstacle'] = ( distance < self.min_distance)

            # elapsed_time = time.time() - start_time
            # if (elapsed_time < self.min_execution_time):
            #     time.sleep(self.min_execution_time - elapsed_time)
            # self.current_threads_fps[self.__class__.__name__] = 1/(time.time()-start_time)
            # start_time = time.time()


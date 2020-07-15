import timer

class ObstacleDetector:
    def __init__(self, min_distance, distance_sensor, car_state):
        self.sensor = distance_sensor
        self.cat_state = car_state
        self.min_distance = min_distance
    
    def __enter__(self):
        """ Entering a with statement """
        self.startThread()
        return self
    
    def __exit__(self, exception_type, exception_value, traceback):
        self.stopThread()
        """ Exit a with statement"""

    def startThread(self):
        # start the thread to follow the road
        t = Thread(target=self._run, name="ObstacleDetector", args=())
        t.start()
        return self

    def stopThread(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def _run(self):
        start_time = time.time()
        while not self.stopped:
            self.car_state['stop_flags']['obstacle'] = ( self.sensor.getDistance() < self.min_distance)
            stop_time = time.time()
            print(f"ObstacleDetector: {1/(stop_time-start_time)} FPS")


from threading import Timer


class _ITrafficSignProcessor:
    present = False
    def __init__(self, conf):
        self.label = self.__class__.__name__
        self.detection_distance = conf['OBJECT_DETECTION']['detection_distance']
        self.cam_focal_lenght = conf['CAMERA']['focal_lenght']
        self.cam_sensor_shape = conf['CAMERA']['sensor_shape']
        self.obj_height = conf['OBJECT_DETECTION']['obj_height'][self.label]
        self.img_height = conf['CAMERA']['parameters']['resolution'][1]

    def set_car_state(self, car_state):
        pass

    def is_nearby(self, obj):
        # default: if a sign is 10% of the height of frame
        obj_height_px = obj.bounding_box[1][1]  - obj.bounding_box[0][1]
        distance = (self.cam_focal_lenght*self.obj_height*self.img_height)/(obj_height_px*self.cam_sensor_shape[1])
        return distance <= self.detection_distance

class Battery(_ITrafficSignProcessor):
    def set_car_state(self, car_state):
        if self.present:
            print("Battery charging station -> Not implemented")

class TrafficLight(_ITrafficSignProcessor):
    def __init__(self, conf, color):
        _ITrafficSignProcessor.__init__(self, conf)
        self.color = color
        self.label += str(color)
        self.no_light_count = 0
        self.max_no_light_gap = 5

    def set_car_state(self, car_state):
        if self.color == 'Red':
            if self.present:
                self.no_light_count = 0
                car_state['stop_flags']['red_light'] = True
            else:
                if self.no_light_count >= self.max_no_light_gap:
                    if car_state['stop_flags']['red_light'] == True: 
                        car_state['stop_flags']['red_light'] = False
                else:
                    self.no_light_count +=1
        elif self.color == 'Green' or self.color == 'Off':
            pass
            
                

class SpeedLimit(_ITrafficSignProcessor):

    def __init__(self, conf, speed_limit):
        _ITrafficSignProcessor.__init__(self, conf)
        self.speed_limit = speed_limit
        self.label += str(speed_limit)

    def set_car_state(self, car_state):
        if self.present:
            car_state['speed_limit'] = self.speed_limit


class StopSign(_ITrafficSignProcessor):
    """
    Stop Sign object would wait
    """
    def __init__(self, conf):
        _ITrafficSignProcessor.__init__(self, conf)
        self.wait_time_in_sec = conf['OBJECT_DETECTION']['stop_sign_wait_time']
        self.max_no_stop_gap = 3 #number of frame that the object detection engin can lose the detection of the stop sign
        self.timer = None
        self._reset()


    def set_car_state(self, car_state):
        if not self.present and self.has_stopped:
            if self.no_stop_count >= self.max_no_stop_gap:
                self._reset()
                car_state['stop_flags']['stop_sign'] = False
                return
            else:
                #it is considered that the sign is still present
                self.no_stop_count += 1
                self.present = True
        else:
            self.no_stop_count = 0

        if self.present:
            if not self.has_stopped:
                car_state['stop_flags']['stop_sign'] = True
                self.has_stopped = True
                self.timer = Timer(self.wait_time_in_sec, self.wait_done, [car_state])
                self.timer.start()
                return

    def wait_done(self, car_state):
        car_state['stop_flags']['stop_sign'] = False

    def _reset(self):
        self.has_stopped = False
        self.no_stop_count = 0
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
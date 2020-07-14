from threading import Timer
import logging


class _TrafficSignProcessor:
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
        distance = (self.cam_focal_lenght*self.obj_height*self.img_height)/(obj_height_px*self.cam_sensor_shape)
        return distance <= self.detection_distance

class Battery(_TrafficSignProcessor):
    def set_car_state(self, car_state):
        print("Battery managment -> Not implemented")

class TrafficLight(_TrafficSignProcessor):
    def __init__(self, conf, color):
        _TrafficSignProcessor.__init__(self, conf)
        self.color = color
        self.label += str(color)

    def set_car_state(self, car_state):
        if self.color == 'red':
            car_state['stop_flags']['red_light'] = True
        elif self.color == 'green' or self.color == 'off':
            car_state['stop_flags']['red_light'] = False


class SpeedLimit(_TrafficSignProcessor):

    def __init__(self, conf, speed_limit):
        _TrafficSignProcessor.__init__(self, conf)
        self.speed_limit = speed_limit
        self.label += str(speed_limit)

    def set_car_state(self, car_state):
        car_state['speed_limit'] = self.speed_limit


class StopSign(_TrafficSignProcessor):
    """
    Stop Sign object would wait
    """
    def __init__(self, conf):
        _TrafficSignProcessor.__init__(self, conf)
        self.wait_time_in_sec = conf['OBJECT_DETECTION']['stop_sign_wait_time']
        self.max_no_stop_gaps = 3 #number of frame that the object detection engin can lose the detection of the stop sign
        self._reset()


    def set_car_state(self, car_state):
        if not self.present and self.has_stopped:
            if self.no_stop_count >= self.max_no_stop_gaps:
                self._reset()
            else:
                #it is considered that the sign is still present
                self.no_stop_count += 1
                self.present = True
        else:
            self.no_stop_count = 0

        if self.in_wait_mode:
            logging.debug('stop sign: 2) still waiting')
            # wait for 2 second before proceeding
            car_state['speed'] = 0
            return

        if not self.has_stopped:
            logging.debug('stop sign: 1) just detected')

            car_state['speed'] = 0
            self.in_wait_mode = True
            self.has_stopped = True
            self.timer = Timer(self.wait_time_in_sec, self.wait_done)
            self.timer.start()
            return

    def wait_done(self):
        logging.debug('stop sign: 3) finished waiting for %d seconds' % self.wait_time_in_sec)
        self.in_wait_mode = False

    def _reset(self):
        self.has_stopped = False
        self.no_stop_count = 0
        self.in_wait_mode = False
        self.timer = None
#!/usr/bin/env python3

# ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
# ----------------------------------- Infos -----------------------------------


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import time
import io

class PicameraController(PiCamera):
	# Var to stop the thread
	stopped = False

	def __init__(self, 
		cam_param_dict = {},
		current_threads_fps = None,
		camera_num=0, 
		stereo_mode='none', 
		stereo_decimate=False, 
		resolution=None, 
		framerate=None, 
		sensor_mode=0, 
		led_pin=None, 
		clock_mode='reset', 
		framerate_range=None):
		
		self.current_threads_fps = current_threads_fps
		
		# initialize the camera
		PiCamera.__init__(self, camera_num, stereo_mode, stereo_decimate, resolution, framerate, sensor_mode, led_pin, clock_mode, framerate_range)
		
		# set camera parameters (refer to PiCamera docs)
		for (arg, value) in cam_param_dict:
			setattr(self, arg, value)

		# initialize the stream
		self.rawCapture = PiRGBArray(self, size=self.resolution)
		self.stream = self.capture_continuous(self.rawCapture,
			format="rgb", use_video_port=True)

		self.current_frame = None

	def capture_np(self):
		self.capture(self.rawCapture, format="rgb", use_video_port=True)
		frame_np = self.rawCapture.array
		self.rawCapture.truncate(0)
		return frame_np

	def __enter__(self):
		""" Entering a with statement """
		self.startThread()
		return self		
		
	def __exit__(self, exception_type, exception_value, traceback):
		self.stopThread()
		""" Exit a with statement"""

	def startThread(self):
		# start the thread to read frames from the video stream
		t = Thread(target=self._update, name=self.__class__.__name__, args=())
		t.start()
		# waiting that the first frame is taken and current_frame is not None enymore
		while (self.current_frame is None):
			time.sleep(0.01)
		return self

	def stopThread(self):
		# indicate that the thread should be stopped
		self.stopped = True


	def _update(self):
		# keep looping infinitely until the thread is stopped
		start_time = time.time()
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			img = f.array
			self.rawCapture.truncate(0)
			self.current_frame = img#self.imgRectifier.undistort(img)
			# if the thread indicator variable is set, stop the thread
			# and restor camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.close()
				return
			if self.current_threads_fps is not None:
				self.current_threads_fps[self.__class__.__name__] = 1/(time.time()-start_time)
				start_time = time.time()

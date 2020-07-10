#inspired by https://github.com/jrosebr1/imutils/blob/master/imutils/video/pivideostream.py

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import time
import cv2
import time
import io

class PicameraController(PiCamera):
	def __init__(self, 
		cam_param_dict, 
		camera_num=0, 
		stereo_mode='none', 
		stereo_decimate=False, 
		resolution=None, 
		framerate=None, 
		sensor_mode=0, 
		led_pin=None, 
		clock_mode='reset', 
		framerate_range=None):
		
		# initialize the camera
		PiCamera.__init__(self, camera_num, stereo_mode, stereo_decimate, resolution, framerate, sensor_mode, led_pin, clock_mode, framerate_range)
		
		# set camera parameters (refer to PiCamera docs)
		for (arg, value) in cam_param_dict:
			setattr(self, arg, value)

		# # initialize the stream
		self.rawCapture = PiRGBArray(self, size=self.resolution)
		self.stream = self.capture_continuous(self.rawCapture,
			format="rgb", use_video_port=True)
		# initialize the frame and the variable used to indicate
		# if the thread should be stopped
		self.frame_np = None
		self.current_frame = None
		self.stopped = False

	def startThread(self):
		# start the thread to read frames from the video stream
		t = Thread(target=self._update, name="CameraCapture", args=())
		t.daemon = True
		t.start()
		# waiting that the first frame is taken and current_frame is not None enymore
		while (self.current_frame is None):
			time.sleep(0.01)
		return self

	def capture_np(self):
		self.capture(self.rawCapture, format="rgb", use_video_port=True)
		self.frame_np = self.rawCapture.array
		self.rawCapture.truncate(0)
		return self.frame_np

	def stopThread(self):
		# indicate that the thread should be stopped
		# self.stream.close()
		# self.rawCapture.close()
		# self.close()
		self.stopped = True


	def _update(self):
		# keep looping infinitely until the thread is stopped
		# StartTime = time.time()
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.current_frame = f.array
			self.rawCapture.truncate(0)

			# if the thread indicator variable is set, stop the thread
			# and restor camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.close()
				return
			# StopTime = time.time()
			# print(f"{1/(StopTime-StartTime):.1f}")
			# StartTime = time.time()

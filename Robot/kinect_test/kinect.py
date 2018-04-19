import math
import freenect
#import cv2
import numpy as np
import pdb

class Kinect:
	def __init__(self, dx=5, dy=10):
		self.width = 640
		self.height = 480
		self.depth_lookup_table = np.empty([2048])
		self.dx = dx
		self.dy = dy

		for i in range(len(self.depth_lookup_table)):
			self.depth_lookup_table[i] = self.raw_depth_to_meters(i)

	def get_raw_depth(self):
	    return freenect.sync_get_depth(0)[0]

	def raw_depth_to_meters(self, raw_depth_value):
		if raw_depth_value < 2047:
			return 1.0/(float(raw_depth_value)*-0.0030711016 + 3.3309495161)
		else:
			return 0.0

	def raw_depth_to_world(self, x, y, raw_depth_value):
		fx_d = 1.0 / 5.9421434211923247e+02
		fy_d = 1.0 / 5.9104053696870778e+02
		cx_d = 3.3930780975300314e+02
		cy_d = 2.4273913761751615e+02

		point = np.empty([3])
		depth = self.depth_lookup_table[int(raw_depth_value)]
		point[0] = (x - cx_d)*depth*fx_d
		point[1] = (y - cy_d)*depth*fy_d
		point[2] = depth
		#pdb.set_trace()

		return point

	def get_point_cloud(self, raw_depth):
		p_w = []
		for x in self.dx*np.array(range(self.width/self.dx)):
			for y in self.dy*np.array(range(self.height/self.dy)):
				if raw_depth[y,x] != 0:
					if self.raw_depth_to_meters(raw_depth[y,x]) <= 6:
						p = self.raw_depth_to_world(x,y,raw_depth[y,x])
						p_w.append(p)

		if not p_w:
			p_w = [[0, 0, 0],[0,0,0]]

		p_wmat = np.asarray(p_w)
		return p_wmat

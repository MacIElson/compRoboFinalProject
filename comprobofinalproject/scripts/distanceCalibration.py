#!/usr/bin/env python

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Vector3
import numpy as np
import math
import time
import matplotlib.pyplot as plt

def nothing(x):
	pass

class controller:
	def __init__(self, verbose = False):
		rospy.init_node('comprobofinalproject', anonymous=True)
		cv2.namedWindow('image')

		# most recent raw CV image
		self.cv_image = None


		self.image = None

		self.mouseClick = None
		
		self.bridge = CvBridge()
		
		#subscribe tocamera images
		self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)

				#create on/off switch for robot, defaulted to off
		self.switch = '0 : OFF \n1 : ON'
		cv2.createTrackbar(self.switch, 'image',0,1,nothing)
		cv2.setTrackbarPos(self.switch,'image',0)


	def on_mouse(self,event,x,y,flags,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			print (x,y)
			self.mouseClick = (x,y)

	def recieveImage(self,raw_image):

		if self.image != None:
			return

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
		except CvBridgeError, e:
			print e
				   
		#display image recieved
		cv2.imshow('Video1', self.cv_image)

		s = cv2.getTrackbarPos(self.switch,'image')
		
		#determine if robot should move
		if s == 1:
			print "Image Selected"
			self.image = self.cv_image
		else:
			pass
		
		#wait for openCV elements to refresh
		cv2.waitKey(3)



	def calibrate(self):
		cv2.destroyAllWindows()
		cv2.namedWindow('imageSelected')
		cv2.setMouseCallback('imageSelected',self.on_mouse)
		cv2.imshow('imageSelected', self.image)

		cv2.waitKey(3)

		offsetDist = 14.5
		distList1 = [24.5,26.5,28.5,30.5,34.5,38.5,44.5]
		distList = [10,12,14,16,20,24,30]
		pixList = []

		for dist in distList:
			print "click on line at " + str(dist) + " cm"
			while self.mouseClick == None:
				time.sleep(.1)
				cv2.waitKey(3)
				click = self.mouseClick
			print click
			self.mouseClick = None
			pixList.append(click[1])

		z = np.polyfit(pixList, distList1,  3)
		print z
		p = np.poly1d(z)
		xp = np.linspace(0, 640, 640)
		_ = plt.plot(pixList, distList1, '.', xp, p(xp), '-')
		plt.ylim(0,50)

		plt.show()
		self.image = None

def main(args):
	# initialize driver
	ic = controller(False)

	#set ROS refresh rate
	r = rospy.Rate(60)

	#keep program running until shutdown
	while not(rospy.is_shutdown()):
		r.sleep()
		if ic.image != None:
			ic.calibrate()

	#close all windows on exit
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
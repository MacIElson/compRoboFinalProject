#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		self.bridge = CvBridge()
		self.image = None
		#self.found_lines = np.zeros((480, 640,3), np.uint8)
		#self.image = cv2.imread("test.png")

	def update_image(self,msg):
		try:
			self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			pass
		except CvBridgeError, e:
			print e

	def two_cluster_lines(self, data):
		pass
	
	def run(self):
		r=rospy.Rate(5)
		while not rospy.is_shutdown():
			if self.image != None:
				frame = self.image
				#cv2.imshow("CAM",frame)	

				# Convert BGR to HSV
				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

				#RED MASKING TAPE HSV
				lower1 = np.array([172,100,100])
				upper1 = np.array([180,255,255])

				mask1 = cv2.inRange(hsv, lower1, upper1)

				lower2 = np.array([-1,100,100])
				upper2 = np.array([13,255,255])
				
				mask2 = cv2.inRange(hsv, lower2, upper2)

				mask = cv2.add(mask1,mask2)
				# Bitwise-AND mask and original image
				res = cv2.bitwise_and(frame,frame, mask= mask)

				

				gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

				cv2.imshow("Filtered",mask)

				edges = cv2.Canny(mask,50,150, apertureSize = 3)
				
				lines = cv2.HoughLines(edges,1,np.pi/180,100)

				pic = frame.copy()
				print lines
				if lines != None:
					for rho,theta in lines[0]:
						a = np.cos(theta)
						b = np.sin(theta)
						x0 = a*rho
						y0 = b*rho
						x1 = int(x0 + 1000*(-b))
						y1 = int(y0 + 1000*(a))
						x2 = int(x0 - 1000*(-b))
						y2 = int(y0 - 1000*(a))
						cv2.line(pic,(x1,y1),(x2,y2),(0,0,255),2)
				
				cv2.imshow("CAM",pic)
				cv2.imshow("EDGES",edges)
				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	

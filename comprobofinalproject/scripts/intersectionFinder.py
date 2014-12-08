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
		self.ignoredBorder = 50

	def update_image(self,msg):
		try:
			self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			pass
		except CvBridgeError, e:
			pass
			#print e

	'''def intersectionCenter(self, corners):
		(x,y) = corners.shape
		totalx = 0.0
		totaly = 0.0
		total = 0
		for i in range(self.ignoredBorder,x-self.ignoredBorder):
			for j in range(self.ignoredBorder,y-self.ignoredBorder):
				if corners[i][j] == 1:
					totalx += float(i)
					totaly += float(j)
					total += 1
		if total!=0:
			return (int(round(totaly/total)),int(round(totalx/total)))
		return (0,0)'''

	def intersectionCenter(self,corners):
		contours, heiarchy = cv2.findContours(corners,1,2)
		if len(contours) > 0:
			totalx = 0.0
			totaly = 0.0
			total = 0
			for contour in contours:
				if cv2.contourArea(contour) > 100:
					M = cv2.moments(contour)
					if M['m00'] == 0:
						print "WARNING: m00 == 0"
					else:
						cx = int(M['m10']/M['m00'])
						cy = int(M['m01']/M['m00'])
						totalx += cx
						totaly += cy
						total += 1
			if(total>0):
				return (int(round(totalx/total)),int(round(totaly/total)))
		return (0,0)
		
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

				corner_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
				dst = cv2.cornerHarris(corner_gray,30,3,0.2)
				dst = cv2.dilate(dst,None)
				corner_pic = frame.copy()

				corner_mask = np.zeros((len(frame),len(frame[0])),np.uint8)

				corner_pic[dst>.01*dst.max()] = [255,0,0]
				corner_mask[dst>.01*dst.max()] = 1
				print len(corner_mask),len(corner_mask[0]),corner_mask.shape
				corner_mask = corner_mask[self.ignoredBorder:corner_mask.shape[0]-self.ignoredBorder, self.ignoredBorder:corner_mask.shape[1]-self.ignoredBorder]


				intersectionPoint = self.intersectionCenter(corner_mask)
				intersectionPoint = (intersectionPoint[0]+self.ignoredBorder,intersectionPoint[1]+self.ignoredBorder)
				cv2.circle(corner_pic,intersectionPoint,2,(0,255,0),2)
				print intersectionPoint
				cv2.imshow("Corners",corner_mask)

				cv2.imshow("frame",corner_pic)

				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	

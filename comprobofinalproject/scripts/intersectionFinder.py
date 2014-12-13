#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from comprobofinalproject.msg import Intersection

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		rospy.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_odometry)
		self.dist_pub = rospy.Publisher("/intersection",Intersection,queue_size = 10)
		self.bridge = CvBridge()
		self.image = None
		self.odometry = None

		self.polynomial = [-5.66680774e-07,5.68132807e-04,-2.17597871e-01,5.62396178e+01]
		self.ignoredBorder = [200,25,50,50]
		self.found = 0
		self.exitBorder = [150,50,50,50]

		self.createTrackbars()

	def update_odometry(self,msg):
		self.odometry = msg

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
	def createTrackbars(self):
		self.name = 'dumb'
		cv2.namedWindow(self.name)
		cv2.createTrackbar('thresh',self.name,0,20,lambda x:x)
		cv2.setTrackbarPos('thresh',self.name,9)


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
			if(total>=2):
				return (int(round(totalx/total)),int(round(totaly/total)))
		return None

	def exitPathCenters(self, border):
		contours, heiarchy = cv2.findContours(border,1,2)
		if len(contours) > 0:
			exits = []
			for contour in contours:
				if cv2.contourArea(contour) > 100:
					M = cv2.moments(contour)
					if M['m00'] == 0:
						print "WARNING: m00 == 0"
					else:
						cx = int(M['m10']/M['m00'])
						cy = int(M['m01']/M['m00'])
						exits.append((cx,cy))
			return exits
		return []
		
	def run(self):
		r=rospy.Rate(20)
		while not rospy.is_shutdown():
			if self.image != None:
				frame = self.image
				
				#RED MASKING TAPE HSV
				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

				lower1 = np.array([172,100,100])
				upper1 = np.array([180,255,255])

				mask1 = cv2.inRange(hsv, lower1, upper1)

				lower2 = np.array([-1,100,100])
				upper2 = np.array([13,255,255])
				
				mask2 = cv2.inRange(hsv, lower2, upper2)

				mask = cv2.add(mask1,mask2)
				
				borderless = mask[self.ignoredBorder[0]:-self.ignoredBorder[1], self.ignoredBorder[2]:-self.ignoredBorder[3]]
				cv2.imshow("mask",borderless)
				#corner_gray = cv2.cvtColor(borderless,cv2.COLOR_BGR2GRAY)
				dst = cv2.cornerHarris(borderless,30,3,0.2)
				dst = cv2.dilate(dst,None)


				shifted_dst = np.zeros((len(frame),len(frame[0])))
				shifted_dst[self.ignoredBorder[0]:-self.ignoredBorder[1], self.ignoredBorder[2]:-self.ignoredBorder[3]] = dst
				corner_pic = frame.copy()


				corner_mask = np.zeros((len(borderless),len(borderless[0])),np.uint8)
				power = cv2.getTrackbarPos('thresh',self.name)
				dst_threshhold = 10**(-power)
				corner_pic[shifted_dst>dst_threshhold] = [255,0,0]
				corner_mask[dst>dst_threshhold] = 1
				print .01*dst.max()


				intersectionPoint = self.intersectionCenter(corner_mask)
				if(intersectionPoint != None):
					self.found += 1
					if(self.found > 4):
						intersectionPoint = (intersectionPoint[0]+self.ignoredBorder[2],intersectionPoint[1]+self.ignoredBorder[0])
						cv2.circle(corner_pic,intersectionPoint,2,(0,255,0),2)

						p = np.poly1d(np.array(self.polynomial))
						dist = p(intersectionPoint[0])

						odom = self.odometry
						x = odom.pose.pose.position.x
						y = odom.pose.pose.position.y
						z = math.atan2(2* (odom.pose.pose.orientation.z * odom.pose.pose.orientation.w),1 - 2 * ((odom.pose.pose.orientation.y)**2 + (odom.pose.pose.orientation.z)**2))
						odom.pose.pose.position.x = x + dist * math.cos(z)
						odom.pose.pose.position.y = y + dist * math.sin(z)


						#Crop mask
						border = mask
						border[self.exitBorder[0]:-self.exitBorder[1],self.exitBorder[2]:-self.exitBorder[3]] = 0
						

						

						exits = self.exitPathCenters(mask)

						for each in exits:
							cv2.line(corner_pic,intersectionPoint,each,(0,255,0),2)


						inter = Intersection(distance = dist,odom = odom)
						#print "Found Intersections:", inter

						self.dist_pub.publish(inter)
				else:
					self.found = 0

				cv2.imshow("frame",corner_pic)
				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	

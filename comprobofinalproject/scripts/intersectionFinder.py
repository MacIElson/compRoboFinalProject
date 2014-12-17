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
import copy
class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		rospy.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_odometry)
		self.dist_pub = rospy.Publisher("/intersection",Intersection,queue_size = 10)
		self.bridge = CvBridge()
		self.image = None
		self.odometry = None

		self.polynomial = [2.64076873e-09,-3.65044029e-06,1.93383304e-03,-4.95872487e-01,7.85179656e+01]
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


	def intersectionCenter(self,corner_mask):
		contours, heiarchy = cv2.findContours(corner_mask,1,2)
		if len(contours) > 0:
			totalx = 0.0
			totaly = 0.0
			total = 0
			corners = []
			for contour in contours:
				if cv2.contourArea(contour) > 300:
					M = cv2.moments(contour)
					if M['m00'] == 0:
						print "WARNING: m00 == 0"
					else:
						cx = int(M['m10']/M['m00'])
						cy = int(M['m01']/M['m00'])
						corners.append((cx,cy))
						totalx += cx
						totaly += cy
						total += 1
			if(total>=2):
				return ((int(round(totalx/total)),int(round(totaly/total))),corners)
		return (None,None)

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

	def exitAngle(self,exit,intersection):
		return math.atan2(intersection[0]-exit[0],float(intersection[1]-exit[1])) 

	def intersection(self,a,b,c,d):
		return (((a[0]*b[1]-a[1]*b[0])*(c[0]-d[0])-(a[0]-b[0])*(c[0]*d[1]-c[1]*d[0]))/((a[0]-b[0])*(c[1]-d[1])-(a[1]-b[1])*(c[0]-d[0])),
			((a[0]*b[1]-a[1]*b[0])*(c[1]-d[1])-(a[1]-b[1])*(c[0]*d[1]-c[1]*d[0]))/((a[0]-b[0])*(c[1]-d[1])-(a[1]-b[1])*(c[0]-d[0])))

	def slope(self,a,b):
		return (math.atan2(a[1]-b[1],float(a[0]-b[0])) + math.pi) % math.pi



	def run(self):
		r=rospy.Rate(20)
		while not rospy.is_shutdown():
			if self.image != None:
				odom = copy.deepcopy(self.odometry)
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

				(intersectionPoint,centers) = self.intersectionCenter(corner_mask)

				if(intersectionPoint != None):
					self.found += 1
					if(self.found >= 4):
						intersectionPoint = (intersectionPoint[0]+self.ignoredBorder[2],intersectionPoint[1]+self.ignoredBorder[0])
						cv2.circle(corner_pic,intersectionPoint,2,(255,0,0),2)


						#Crop mask
						border = mask
						border[self.exitBorder[0]:-self.exitBorder[1],self.exitBorder[2]:-self.exitBorder[3]] = 0
						exits = self.exitPathCenters(mask)

						if(len(exits) == 3 and len(centers) == 2):
							center_slope = self.slope(centers[0],centers[1])
							perp_slope = (center_slope + (math.pi/2)) % math.pi
							#print "slope", center_slope
							#print "perp", perp_slope
							slopes = []
							exits_copy = copy.deepcopy(exits)
							for each in exits_copy:
								#print self.slope(intersectionPoint,each)
								slopes.append(self.slope(intersectionPoint,each))
							index = slopes.index(min(slopes,key = lambda x: min(abs(x - perp_slope),math.pi - abs(x-perp_slope))))
							perp_point = exits_copy.pop(index)
							intersectionPoint = self.intersection(perp_point,intersectionPoint,exits_copy[0],exits_copy[1])
							intersectionPoint = (int(round(intersectionPoint[0])),int(round(intersectionPoint[1])))
							cv2.circle(corner_pic,intersectionPoint,2,(0,255,0),2)

						p = np.poly1d(np.array(self.polynomial))
						dist = p(intersectionPoint[1])

						x = odom.pose.pose.position.x
						y = odom.pose.pose.position.y
						z = math.atan2(2* (odom.pose.pose.orientation.z * odom.pose.pose.orientation.w),1 - 2 * ((odom.pose.pose.orientation.y)**2 + (odom.pose.pose.orientation.z)**2))
						x += (dist/100) * math.cos(z)
						y += (dist/100) * math.sin(z)

						angles = []
						for each in exits:
							angles.append(self.exitAngle(each,intersectionPoint))
							#angles.append(self.exitAngle(each,intersectionPoint))
							cv2.line(corner_pic,intersectionPoint,each,(0,255,0),2)
						angles.remove(max(angles, key = lambda x: abs(x)))
						print angles
						odom_angles = [(z + angle + math.pi) % (2 * math.pi) - math.pi for angle in angles]
						
						print dist
						inter = Intersection(x = x, y = y, z = z, exits = odom_angles, raw_exits=angles ,distance = dist,odom = odom)

						self.dist_pub.publish(inter)
				else:
					self.found = 0

				cv2.imshow("frame",corner_pic)
				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	

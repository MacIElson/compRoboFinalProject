#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np
import math
"""
Parts of this code are modified from a tutorial on feature matching and homography
source: http://docs.opencv.org/trunk/doc/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html#
"""

class SignWatcher:

    def __init__(self):      
        # The value of this determine how sure we are of seeing the stop sign
        # 0 -> no stop sign, 1 -> possible stop sign, 2 -> probably, 3 -> almost definitely  
        self.stop_detected = 0 

        # number of images recieved
        self.image_count = 0
        
        # stop sign image we're looking for
        self.stop_sign_img = cv2.imread("greenSmall.png",0)
        self.sift = cv2.SIFT()

        # minimum matches to say we've found the stopsign 
        self.MIN_MATCH_COUNT = 10

        #key points and decsriptors of the query image
        self.templateKp1, self.templateDes1 = self.sift.detectAndCompute(self.stop_sign_img,None)
        
        # set up feature matching
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
 
        rospy.init_node("sign_found")
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)
        self.sign_found_pub = rospy.Publisher("sign_found", String, queue_size=10)

        # most recent raw CV image
        self.cv_image = None
        self.bridge = CvBridge()
       
        #subscribe to odometry
        rospy.Subscriber('odom',Odometry,self.odometryCb)
        self.xPosition = -1.0
        self.yPosition = -1.0
        self.image_odom = [self.xPosition, self.yPosition] 
    
    # recieves and handles images from the Neato
    def recieveImage(self,raw_image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e

        self.image_count += 1     

        # record the odom, so we can return location at the time the picture was recieved, not when the image is processed, because that can take .2 seconds 
        self.image_odom = [self.xPosition, self.yPosition] 
 
        # only check every 4th image, because each check takes too long       
        if self.image_count % 4 is 0:
            self.checkStop()
        cv2.imshow('Video2', self.cv_image)

        cv2.waitKey(3)

    # checks the most recent image for a stop sign 
    def checkStop(self):
        # find the keypoints and descriptors with SIFT
        imageKP, imageDes = self.sift.detectAndCompute(self.cv_image, None)

        #returns all the matches between the images
        matches = self.flann.knnMatch(self.templateDes1,imageDes,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        #if we have enough good matches, we assume we found the stop sign
        if len(good)>self.MIN_MATCH_COUNT :
            # points on the query image
            src_pts = np.float32([ self.templateKp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            #points on the actual image
            dst_pts = np.float32([ imageKP[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            
            h,w = self.stop_sign_img.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)

            #check if the matching object we've found is a square
            isSquare, meanSide = self.isSquare(dst)          
            if isSquare:  

                #calculate the distance to the stopsign
                stopDist = self.estRange(meanSide)

                # now we have found this is a legitimate match (found object, object is square, square has sides with length over 1 pixel)
                # increment stop_detected. If it is above the threshold, stop sign is found. 
                if self.stop_detected < 3:
                    self.stop_detected += 1
                    if self.stop_detected is 2:
                        print "stop sign found"
                        self.signFound(stopDist)    
                
        else:
            # If there were not enough matches lower the stop_detected by 1. 
            # Don't lower when enough matches, but no square, because often 
            matchesMask = None
            if self.stop_detected > 0:
                self.stop_detected -= 1
                if self.stop_detected is 1:
                    print "stop sign lost"

    # When sign is found, this sends the current odom and the distance to the sign in meters
    def signFound(self, stopDist):
        odom = str(self.image_odom[0]) + ","+ str(self.image_odom[1])
        message = odom + "," + str(0.0254*stopDist)
        self.sign_found_pub.publish(message)
        
    # Estimates the range in inches of the neato to the stop sign
    def estRange(self, pixelDist):
        return  90.015400923886219 + -.58834960136704306 * pixelDist + .0012499950650678758 * math.pow(pixelDist,2)

    # Simply deteremines if the four points are probably a square and returns a bool and the average side length
    def isSquare(self, pts):
        sides = []
        sides.append(pts[1][0][1] - pts[0][0][1])
        sides.append(pts[2][0][0] - pts[1][0][0])
        sides.append(pts[2][0][1] - pts[3][0][1])
        sides.append(pts[3][0][0] - pts[0][0][0])
        
        maxSide = max(sides)
        minSide = min(sides)
        meanSide = np.mean(sides)

        # if the any side if more than 10% over or under the mean or if the sides are less than a pixel long, return false
        if maxSide > meanSide*1.1 or minSide < meanSide *.9 or meanSide < 1:
            return False, meanSide 
        
        return True, meanSide

    # call back for odom that constantly updates our xy pos 
    def odometryCb(self,msg):
        self.xPosition = msg.pose.pose.position.x
        self.yPosition = msg.pose.pose.position.y



def main(args):
    ic = SignWatcher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)




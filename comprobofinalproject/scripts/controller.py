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

#use colorCalibration.py to find the correct color range for the red line
#connect to neato --> run this and StopSignFinder --> turn robot to on

#function that does nothing
def nothing(x):
    pass

class controller:
    def __init__(self, verbose = False):
        rospy.init_node('comprobofinalproject', anonymous=True)
        cv2.namedWindow('image')

        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None
        
        self.bridge = CvBridge()

        #create on/off switch for robot, defaulted to off
        self.switch = '0 : OFF \n1 : ON'
        cv2.createTrackbar(self.switch, 'image',0,1,self.stop)
        cv2.setTrackbarPos(self.switch,'image',0)
        
        #subscribe tocamera images
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)

        #subscribe to odometry
        rospy.Subscriber('odom',Odometry,self.odometryCb)

        #set up publisher to send commands to the robot
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.dprint("Driver Initiated")

    #function that stops the robot is 0 is passed in, primary use is call back from stop switch
    def stop(self, x):
        if x == 0:
            self.sendCommand(0,0)
    
    #odometry callback
    def odometryCb(self,msg):
        self.xPosition = msg.pose.pose.position.x
        self.yPosition = msg.pose.pose.position.y
        
    # callback when image recieved
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
        except CvBridgeError, e:
            print e
                   
        #display image recieved
        cv2.imshow('Video1', self.cv_image)

        #retreive value of on/off switch
        s = cv2.getTrackbarPos(self.switch,'image')
        
        #determine if robot should move
        if s == 0:
            self.dprint("Driving Stopped")
        else:
            pass
        
        #wait for openCV elements to refresh
        cv2.waitKey(3)

    #send movement command to robot
    def sendCommand(self, lin, ang):
        twist = Twist()
        twist.linear.x = lin; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang
        self.pub.publish(twist)

    #function that makes print statements switchable
    def dprint(self, print_message):
        if self.verbose:
            print print_message

def main(args):
    # initialize driver
    ic = controller(False)

    #set ROS refresh rate
    r = rospy.Rate(60)

    #keep program running until shutdown
    while not(rospy.is_shutdown()):
        r.sleep()

    #close all windows on exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

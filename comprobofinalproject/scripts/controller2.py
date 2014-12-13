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

import copy

from PIDcontroller import PID

#use colorCalibration.py to find the correct color range for the red line
#connect to neato --> run this and StopSignFinder --> turn robot to on

class controller:
    def __init__(self, verbose = False):
        rospy.init_node('comprobofinalproject', anonymous=True)
        cv2.namedWindow('image')

        # if true, we print what is going on
        self.verbose = verbose

        # most recent raw CV image
        self.cv_image = None
        self.newImage = False
        
        self.bridge = CvBridge()

        self.createTrackbars()
        
        #subscribe tocamera images
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.recieveImage)

        #subscribe to odometry
        rospy.Subscriber('odom',Odometry,self.odometryCb)
        self.newOdom = False
        self.xPosition = None
        self.yPosition = None

        #set up publisher to send commands to the robot
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.signDetected = False
        self.intersectionDetected = False

        self.lineFollowingOn = True
        self.initializeLineFollowPID()

        cv2.waitKey(3)

        self.dprint("Driver Initiated")

    def createTrackbars(self):
        #create on/off switch for robot, defaulted to off
        self.switchC = 'Controller \n0 : OFF \n1 : ON'
        cv2.createTrackbar(self.switchC, 'image',0,1,self.stop)
        cv2.setTrackbarPos(self.switchC,'image',0)

        self.switchM = 'sendCommand \n 0 : OFF \n1 : ON'
        cv2.createTrackbar(self.switchM, 'image',0,1,self.stop)
        cv2.setTrackbarPos(self.switchM,'image',1)

        cv2.createTrackbar('speed','image',0,200,nothing)
        cv2.setTrackbarPos('speed','image',15)

        cv2.createTrackbar('pidP','image',0,8000,nothing)
        cv2.setTrackbarPos('pidP','image',130)

        cv2.createTrackbar('pidI','image',0,400,nothing)
        cv2.setTrackbarPos('pidI','image',4)

        cv2.createTrackbar('pidD','image',0,4000,nothing)
        cv2.setTrackbarPos('pidD','image',20)

        cv2.createTrackbar('lowH','image',0,255,nothing)
        cv2.setTrackbarPos('lowH','image',0)
        cv2.createTrackbar('lowS','image',0,255,nothing)
        cv2.setTrackbarPos('lowS','image',156)
        cv2.createTrackbar('lowV','image',0,255,nothing)
        cv2.setTrackbarPos('lowV','image',87)
        cv2.createTrackbar('highH','image',0,255,nothing)
        cv2.setTrackbarPos('highH','image',255)
        cv2.createTrackbar('highS','image',0,255,nothing)
        cv2.setTrackbarPos('highS','image',255)
        cv2.createTrackbar('highV','image',0,255,nothing)
        cv2.setTrackbarPos('highV','image',169)

    def mainloop(self):
        if cv2.getTrackbarPos(self.switchC,'image') == 1:
            self.newOdomTemp = self.newOdom
            self.newImageTemp = self.newImage
            self.newOdom = False
            self.newImage = False
            self.xPositionTemp = self.xPosition
            self.yPositionTemp = self.yPosition
            self.cv_imageTemp = copy.copy(self.cv_image)

            if self.signDetected:
                pass
            if self.intersectionDetected:
                pass
            if self.newImageTemp and self.lineFollowingOn:
                self.lineFollow()

    def lineFollow(self):
        smallCopy = self.cv_imageTemp[350:480]

        hsv = cv2.cvtColor(smallCopy, cv2.COLOR_BGR2HSV)

        lowH = cv2.getTrackbarPos('lowH','image')
        lowS = cv2.getTrackbarPos('lowS','image')
        lowV = cv2.getTrackbarPos('lowV','image')
        highH = cv2.getTrackbarPos('highH','image')
        highS = cv2.getTrackbarPos('highS','image')
        highV = cv2.getTrackbarPos('highV','image')

        lower_red = np.array([lowH,lowS,lowV])
        upper_red = np.array([highH,highS,highV])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        cv2.imshow('mask', mask)

        #sum all coplumns into 1 row
        driveRow = np.sum(mask,0)
        
        #initialize array of x indicies where the road exists
        num = []

        #fill array num with x indicies where the road exists
        for i in range(len(driveRow)):
            if driveRow[i] > 0:
                num.append(i+1)

        try:
            averageLineIndex = (float(sum(num))/len(num))
            print "averageLineIndex: " + str(averageLineIndex)
        except:
            print "no line found"
            return

                #get and set PID control constants
        pidP100 = cv2.getTrackbarPos('pidP','image')
        pidI100 = cv2.getTrackbarPos('pidI','image')
        pidD100 = cv2.getTrackbarPos('pidD','image')

        pidP = float(pidP100)/100
        pidI = float(pidI100)/100
        pidD = float(pidD100)/100

        self.pid.setKp(pidP)
        self.pid.setKi(pidI)
        self.pid.setKd(pidD)

        #use the pid controller to determine the anglular velocity
        ang = self.pid.update(averageLineIndex)/1000

        speed = cv2.getTrackbarPos('speed','image')/100.0

        self.sendCommand(speed, ang)
        

    def initializeLineFollowPID(self):
        self.pid = PID(P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500)
        self.pid.setPoint(float(320))

    #function that stops the robot is 0 is passed in, primary use is call back from stop switch
    def stop(self, x):
        if x == 0:
            self.sendCommand(0,0)
    
    #odometry callback
    def odometryCb(self,msg):
        self.xPosition = msg.pose.pose.position.x
        self.yPosition = msg.pose.pose.position.y
        self.newOdom = True
        
    # callback when image recieved
    def recieveImage(self,raw_image):
        self.dprint("Image Recieved")

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")
            self.newImage = True
        except CvBridgeError, e:
            print e
                   
        #display image recieved
        cv2.imshow('Video1', self.cv_image)

        cv2.waitKey(3)

    #send movement command to robot
    def sendCommand(self, lin, ang):
        print "speed: " + str(lin) + ", " + "ang: " + str(ang)
        if cv2.getTrackbarPos(self.switchM,'image') == 1:
            twist = Twist()
            twist.linear.x = lin; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang
            self.pub.publish(twist)

    #function that makes print statements switchable
    def dprint(self, print_message):
        if self.verbose:
            print print_message

def nothing(x):
    pass

def main(args):
    # initialize driver
    ic = controller(False)

    #set ROS refresh rate
    r = rospy.Rate(30)

    #keep program running until shutdown
    while not(rospy.is_shutdown()):
        ic.mainloop()
        r.sleep()

    #close all windows on exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

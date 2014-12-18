#!/usr/bin/env python

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import uuid

import numpy as np
import math

import random

from comprobofinalproject.srv import *

global nodeGraph = {}

class DecisionMaker:
    def __init__(self):
        rospy.init_node('getTurn')
        rospy.Subscriber("task", String, self.taskChange)
        s = rospy.Service('getTurn', intersectionFoundGetTurn, self.handleNewIntersection)
        self.mapBuilder = MapBuilder()
        print "Ready to pick Direction."

    def taskChange(self,msg):
        self.task = msg

    def handleNewIntersection(self,msg):
        if self.task == "Map":
            self.mapBuilder.incomingIntersection(msg)
        elif self.task == "Path":
            pass
        else:
            ranInt = random.randrange(0,len(msg.exits))
            return intersectionFoundGetTurnResponse(exit_chosen = msg.exits[ranInt],exit_chosen_raw = msg.exits_raw[ranInt])

class MapBuilder:
    def __init__(self):
        global nodeGraph
        nodeGraph = {}
        self.currentIntersectionUUID = None
        self.previousDirection = None

    def incomingIntersection(self,msg):
        global nodeGraph

        self.previousIntersectionUUID = self.currentIntersectionUUID
        self.previousDirection = self.currentDirection 

        newIntersection = self.createDetailedIntersectionFromMsg(msg)

        if len(nodeGraph) == 0:
            nodeGraph[newIntersection.uuid] = newIntersection
            self.currentIntersectionUUID = newIntersection.uuid
        else:
            known = isKnownIntersection(newIntersection)
            if known == False:
                addNewIntersectionToMap(self.previousIntersectionUUID, self.previousDirection, newIntersection,msg)
            else:
                self.currentIntersectionUUID = known
                updateMap(self.previousIntersectionUUID, self.previousDirection, self.currentIntersectionUUID)

        currentIntersection = nodeGraph[self.currentIntersectionUUID]
        if len(currentIntersection.getNotVisitedExits()) == 0:
            
        else:


        self.currentDirection = (value,index)

    def addNewIntersectionToMap(self,previousUUID,previousDirection,newIntersection,msg):
        global nodeGraph
        nodeGraph[newIntersection.uuid] = newIntersection
        exits = nodeGraph[previousUUID].exits
        nodeGraph[previousUUID].exits[previousDirection[1]].connectedNodeId = newIntersection.uuid
        nodeGraph[newIntersection.uuid].exits[len(exits-1)].connectedNodeId = previousUUID

    def updateMap(self,previousUUID,previousDirection,currentUUID,msg):
        global nodeGraph
        nodeGraph[previousUUID].exits[previousDirection[1]].connectedNodeId = currentUUID        
        exits = nodeGraph[currentUUID].exits
        closestExitIndex = min(range(len(exits)), key=lambda i: abs(math.atan2(math.sin(exits[i]-msg.current_path_exit), math.cos(exits[i]-msg.current_path_exit))))
        nodeGraph[newIntersection.uuid].exits[len(exits-1)].connectedNodeId = previousUUID

    def isKnownIntersection(self,intersection):
        for uuid, knownIntersection in d.iteritems():
            dist = euclidDistance(intersection.x,intersection.y,knownIntersection.x,knownIntersection.y)
            if dist < .10:
                return uuid
        return False

    #calculate distance between 2 points
    def euclidDistance(self,x1,y1,x2,y2):
        return math.hypot(x2 - x1, y2 - y1)

    def createDetailedIntersectionFromMsg(self,msg):
        exitList = []
        for rawExit in msg.exits:
            exitList.append(Exit(rawExit))

        exitList.append(Exit(msg.current_path_exit))
        newI = DetailedIntersection(uuid.uuid4(),msg.x,msg.y,exitList)
        return newI

class DetailedIntersection:
    def __init__(self,uuid,x,y,exits):
        self.uuid = uuid
        self.x = x
        self.y = y
        self.exits = exits

    def getNotVisitedExits(self):
        exitList = []
        for exit in exits:
            if not exit.isVisited:
                exitList.append(exit)
        return exitList

class Exit:
    def __init__(self,angle,connectedNodeId = None):
        self.angle = angle
        self.connectedNodeId = connectedNodeId

    def isVisited(self):
        if self.connectedNodeID == None:
            return False
        else:
            return True


def main(args):
    # initialize driver
    ic = DecisionMaker()

    #set ROS refresh rate
    r = rospy.Rate(90)

    #keep program running until shutdown
    while not(rospy.is_shutdown()):
        r.sleep()

    #close all windows on exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
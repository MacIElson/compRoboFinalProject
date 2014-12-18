#!/usr/bin/env python

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import uuid

import numpy as np
import pylab as pl
import math

from matplotlib import collections  as mc

import random
from std_msgs.msg import String
import matplotlib.pyplot as plt
from comprobofinalproject.srv import *

global nodeGraph

class DecisionMaker:
    def __init__(self):
        rospy.init_node('getTurn')
        self.task = "Random"
        rospy.Subscriber("task", String, self.taskChange)
        rospy.Subscriber("mapVisual", String, self.mapVisual)
        s = rospy.Service('getTurn', intersectionFoundGetTurn, self.handleNewIntersection)
        self.mapBuilder = MapBuilder()
        print "Ready to pick Direction."

    def taskChange(self,msg):
        self.task = msg.data

    def mapVisual(self,msg):
        global nodeGraph
        self.visualMap = VisualMap(nodeGraph)

    def handleNewIntersection(self,msg):
        if self.task == "Map":
            direction = self.mapBuilder.incomingIntersection(msg)
            return intersectionFoundGetTurnResponse(exit_chosen = direction)
        elif self.task == "Path":
            pass
        else:
            ranInt = random.randrange(0,len(msg.exits))
            return intersectionFoundGetTurnResponse(exit_chosen = msg.exits[ranInt])

class VisualMap:
    def __init__(self,nodeGraph):
        self.nodeGraph = nodeGraph
        self.plotGraph()

    def plotGraph(self):
        xPoints = []
        yPoints = []
        lines = []

        for uuid, knownIntersection in self.nodeGraph.iteritems():
            xPoints.append(knownIntersection.x)
            yPoints.append(knownIntersection.y)
            for exit in knownIntersection.getVisitedExits():
                lines.append([(knownIntersection.x,knownIntersection.y),(nodeGraph[exit.connectedNodeId].x,nodeGraph[exit.connectedNodeId].y)])

        lc = mc.LineCollection(lines, linewidths=2)
        fig, ax = pl.subplots()
        pl.plot(xPoints, yPoints, 'ro')
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        pl.show()


class MapBuilder:
    def __init__(self):
        global nodeGraph
        nodeGraph = {}
        self.currentIntersectionUUID = None
        self.previousDirection = None
        self.currentDirection = None

    def incomingIntersection(self,msg):
        global nodeGraph

        self.previousIntersectionUUID = self.currentIntersectionUUID
        self.previousDirection = self.currentDirection 

        newIntersection = self.createDetailedIntersectionFromMsg(msg)

        if len(nodeGraph) == 0:
            print "added node to empty graph"
            nodeGraph[newIntersection.uuid] = newIntersection
            self.currentIntersectionUUID = newIntersection.uuid
        else:
            known = self.isKnownIntersection(newIntersection)
            if known == False:
                print "adding new node to graph"
                self.currentIntersectionUUID = newIntersection.uuid
                self.addNewIntersectionToMap(self.previousIntersectionUUID, self.previousDirection, newIntersection,msg)
            else:
                print "updating location to existing node"
                self.currentIntersectionUUID = known
                self.updateMap(self.previousIntersectionUUID, self.previousDirection, self.currentIntersectionUUID,msg)

        currentIntersection = nodeGraph[self.currentIntersectionUUID]
        notVisitedExits = currentIntersection.getNotVisitedExits()
        if len(notVisitedExits) == 0:
            print "All Exits Visited"
            ranInt = random.randrange(0,len(currentIntersection.exits))
            direction = currentIntersection.exits[ranInt].angle
            self.currentDirection = (direction,ranInt)
        else:
            print "Going To Unvisited Exit"
            chosenExit = random.choice(notVisitedExits)
            chosenIndex = currentIntersection.exits.index(chosenExit)
            self.currentDirection = (chosenExit.angle,chosenIndex)

        print "Sending direction: " + str(self.currentDirection)
        return self.currentDirection[0]

    def addNewIntersectionToMap(self,previousUUID,previousDirection,newIntersection,msg):
        global nodeGraph
        nodeGraph[newIntersection.uuid] = newIntersection
        exits = nodeGraph[previousUUID].exits
        nodeGraph[previousUUID].exits[previousDirection[1]].connectedNodeId = newIntersection.uuid
        nodeGraph[newIntersection.uuid].exits[-1].connectedNodeId = previousUUID

    def updateMap(self,previousUUID,previousDirection,currentUUID,msg):
        global nodeGraph
        nodeGraph[previousUUID].exits[previousDirection[1]].connectedNodeId = currentUUID        
        exits = nodeGraph[currentUUID].exits
        closestExitIndex = min(range(len(exits)), key=lambda i: abs(math.atan2(math.sin(exits[i].angle-msg.current_path_exit), math.cos(exits[i].angle-msg.current_path_exit))))
        nodeGraph[currentUUID].exits[closestExitIndex].connectedNodeId = previousUUID

    def isKnownIntersection(self,intersection):
        global nodeGraph
        for uuid, knownIntersection in nodeGraph.iteritems():
            dist = self.euclidDistance(intersection.x,intersection.y,knownIntersection.x,knownIntersection.y)
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
        for exit in self.exits:
            if not exit.isVisited():
                exitList.append(exit)
        return exitList

    def getVisitedExits(self):
        exitList = []
        for exit in self.exits:
            if exit.isVisited():
                exitList.append(exit)
        return exitList

class Exit:
    def __init__(self,angle,connectedNodeId = None):
        self.angle = angle
        self.connectedNodeId = connectedNodeId

    def isVisited(self):
        if self.connectedNodeId == None:
            return False
        else:
            return True


def main(args):
    # initialize driver
    ic = DecisionMaker()

    #set ROS refresh rate
    r = rospy.Rate(30)

    #keep program running until shutdown
    while not(rospy.is_shutdown()):
        r.sleep()

    #close all windows on exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
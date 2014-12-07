#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap
import heapq

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
from std_msgs.msg import Header

class odomPathNode:
	def __init__(self):
		rospy.init_node('odom_path_node', anonymous=True)
		self.path = Path(poses = [],header = Header(stamp=rospy.Time.now(),frame_id="odom"))
		self.pathPub = rospy.Publisher('/odom_path', Path, queue_size=10)
		rospy.Subscriber('odom',Odometry,self.updatePath)

	def updatePath(self,msg):
		self.path.poses.append(PoseStamped(pose = msg.pose.pose,header=Header(stamp=rospy.Time.now(),frame_id="odom")))
		self.pathPub.publish(self.path)

	def run(self):
		r = rospy.Rate(10)
		while not(rospy.is_shutdown()):
			r.sleep()

if __name__ == "__main__":
	node = odomPathNode()
	node.run()

#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import sys, traceback, math, copy, random
import numpy as np
import tf

from visualization_msgs.msg import MarkerArray, Marker
from hector_worldmodel_msgs.msg import ObjectModel

from geometry_msgs.msg import PoseStamped

from gki_sickrd_task.actions import Actions

def xy_distance(point1, point2):
	return math.hypot(point1.x - point2.x, point1.y - point2.y)

def worldmodel_cb(msg):
	#print msg
	pass

# actions
def move_to(stamped):
	pass

if __name__ == "__main__":
	rospy.init_node("sickrd_task", log_level=rospy.INFO)
	tf_listener = tf.TransformListener()
	
	# input
	worldmodel_subscriber = rospy.Subscriber('/worldmodel/objects', ObjectModel, callback=worldmodel_cb)
	# TODO: barcode reader
	
	actions = Actions()
	
	# visualization
	visualization_publisher = rospy.Publisher('/task/visualization_marker_array', MarkerArray, latch=True)

	# strategy

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
	


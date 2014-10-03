#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import sys, traceback, math, copy, random
import numpy as np
import tf

from visualization_msgs.msg import MarkerArray, Marker
from gki_sickrd_task.actions import Actions
from gki_sickrd_task.worldmodel import Worldmodel
from gki_sickrd_task.random_move_strategy import RandomMoveStrategy

# def xy_distance(point1, point2):
# 	return math.hypot(point1.x - point2.x, point1.y - point2.y)

if __name__ == "__main__":
	rospy.init_node("sickrd_task", log_level=rospy.INFO)
	tf_listener = tf.TransformListener()
	viz_pub = rospy.Publisher('/task/visualization_marker_array', MarkerArray, latch=True)
	actions = Actions(tf_listener, viz_pub)
	worldmodel = Worldmodel(tf_listener, viz_pub)
	
	# strategy
	strategy = RandomMoveStrategy(actions, worldmodel)
	strategy.decide()
	
	rospy.spin()
	


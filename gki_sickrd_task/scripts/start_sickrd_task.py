#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import sys, traceback, math, copy, random
import numpy as np
import tf

from gki_sickrd_task.params import Params
from gki_sickrd_task.actions import Actions
from gki_sickrd_task.percepts import Percepts
from gki_sickrd_task.random_move_strategy import RandomMoveStrategy
from gki_sickrd_task.deliver_cubes_strategy import DeliverCubesStrategy

# def xy_distance(point1, point2):
# 	return math.hypot(point1.x - point2.x, point1.y - point2.y)

if __name__ == "__main__":
	rospy.init_node("sickrd_task")
	Params.update()
	actions = Actions()
	percepts = Percepts()
	
	# strategy
	#strategy = RandomMoveStrategy(actions, percepts)
	strategy = DeliverCubesStrategy(actions, percepts)
	decision_timer = rospy.Timer(rospy.Duration(1.0), strategy.decide)
	
	
	rospy.spin()
	


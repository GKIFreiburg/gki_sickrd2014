#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math

from hector_worldmodel_msgs.msg import ObjectModel, Object, ObjectInfo
from gki_sickrd_task.actions import Actions

class DetectAllBannersStrategy(object):
	def __init__(self, actions):
		self.actions = actions
		self.worldmodel = None
		self.visited = set()
	
	def worldmodel_cb(self, msg):
		self.worldmodel = msg
	
	def decide(self):
		if not self.worldmodel:
			return
		
		if len(self.visited) >= 10:
			rospy.loginfo('finished.')
		
		not_visited = set()
		for object in self.worldmodel.objects:
			if object not in visited:
				not_visited.add(object)
		
		# explore
		if len(not_visited) == 0:
			rospy.loginfo('exploring...')
			# actions.
			return
		
		# approach
		for object in not_visited:
			# find closest to robot
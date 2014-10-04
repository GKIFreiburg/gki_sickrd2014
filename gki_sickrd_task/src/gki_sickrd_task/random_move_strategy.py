#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math

from gki_sickrd_task.actions import Actions
from gki_sickrd_task.worldmodel import Worldmodel
from gki_sickrd_task.tools import Tools

class RandomMoveStrategy(object):
	def __init__(self, actions, worldmodel):
		self.actions = actions
		self.worldmodel = worldmodel
		self.tools = Tools()
		self.previous_scan_pose = None
		

	def decide(self):
		try:
			self.worldmodel.estimate_center_from_map()
			self.worldmodel.estimate_center_from_worldmodel()
		
			# scan
			if not self.previous_scan_pose:
				rospy.loginfo('initial sweeping...')
				self.actions.camera_sweep(self.sweep_done_cb)
				return
				
			current_pose = self.tools.get_current_pose()
			if self.tools.xy_distance(self.previous_scan_pose, current_pose) > 2.0:
				rospy.loginfo('sweeping...')
				self.actions.stop()
				self.actions.camera_sweep(self.sweep_done_cb)
				return
			
			# explore
			rospy.loginfo('exploring...')
			exploration_pose = self.worldmodel.sample_scan_pose()
			self.actions.move_to(exploration_pose, self.move_done_cb, self.move_timeout_cb)
			
		except Exception as e:
			rospy.logwarn('exception: '+e.message)
			rospy.sleep(1.0)
			self.decide()
		
	def sweep_done_cb(self, status, result):
		rospy.loginfo('camera_sweep: done')
		self.previous_scan_pose = self.tools.get_current_pose()
		self.actions.look_to(self.look_done_cb)

	def look_done_cb(self, status, result):
		rospy.loginfo('camera_look: done')
		self.decide()

	def move_done_cb(self, status, result):
		rospy.loginfo('random_move: done')
		self.actions.cancel_move_timeout()
		self.decide()

	def move_timeout_cb(self, event):
		rospy.loginfo('random_move: timeout')
		self.decide()
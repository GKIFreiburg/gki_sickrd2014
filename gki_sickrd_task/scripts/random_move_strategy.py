#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import threading

from gki_sickrd_task.params import Params
from gki_sickrd_task.actions import Actions
from gki_sickrd_task.percepts import Percepts
from gki_sickrd_task.tools import Tools

class RandomMoveStrategy(object):
	def __init__(self):
		self.actions = Actions()
		self.percepts = Percepts()
		self.tools = Tools()
		self.previous_scan_pose = None
		self.decision_required = True		
		# align camera
		self.actions.look_to(self.look_done_cb)

	def decide(self, event):
		if not self.decision_required:
			return 
		try:
			Params.update()
			self.percepts.estimate_center_from_map()
			#self.percepts.estimate_center_from_worldmodel()
		
			# scan
			if not self.previous_scan_pose:
				rospy.loginfo('initial sweeping...')
				self.actions.camera_sweep(self.sweep_done_cb)
				self.decision_required = False
				return
				
			current_pose = self.tools.get_current_pose()
			if self.tools.xy_distance(self.previous_scan_pose, current_pose) > Params.get().minimum_travel_distance_for_rescan:
				rospy.loginfo('sweeping...')
				self.actions.stop()
				self.actions.camera_sweep(self.sweep_done_cb)
				self.decision_required = False
				return
			
			# explore
			rospy.loginfo('exploring...')
			exploration_pose = self.percepts.sample_scan_pose()
			self.actions.move_to(exploration_pose, self.move_done_cb, self.move_timeout_cb)
			self.decision_required = False
		except Exception as e:
			rospy.logwarn('exception {}: {}'.format(type(e), e.message))
			rospy.sleep(1.0)

	def sweep_done_cb(self, status, result):
		rospy.loginfo('camera_sweep: done')
		self.previous_scan_pose = self.tools.get_current_pose()
		self.actions.look_to(self.look_done_cb)
		self.decision_required = True

	def look_done_cb(self, status, result):
		rospy.loginfo('camera_look: done')
		self.decision_required = True

	def move_done_cb(self, status, result):
		rospy.loginfo('random_move: done')
		self.actions.cancel_move_timeout()
		self.decision_required = True

	def move_timeout_cb(self, event):
		rospy.loginfo('random_move: timeout')
		self.decision_required = True

if __name__ == "__main__":
	rospy.init_node("sickrd_task")
	
	# strategy
	strategy = RandomMoveStrategy()
	decision_timer = rospy.Timer(rospy.Duration(1.0), strategy.decide)
	rospy.spin()


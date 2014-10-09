#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy

class Params(object):
	instance = None

	@staticmethod
	def get():
		if Params.instance == None:
			Params.instance = Params()
		return Params.instance

	@staticmethod
	def update():
		if Params.instance == None:
			Params.instance = Params()
		Params.instance._update()

	def __init__(self):
		self._update()

	def _update(self):
		#rospy.loginfo('updating parameters.')
		self.max_loading_station_distance_from_center = rospy.get_param('max_loading_station_distance_from_center', 2.0)
		self.optimal_exploration_distance = rospy.get_param('optimal_exploration_distance', 4.0)
		self.approach_distance = rospy.get_param('approach_distance', 1.0)
		self.ring_distance = rospy.get_param('ring_distance', 0.3)
		self.min_travel_distance_for_rescan = rospy.get_param('min_travel_distance_for_rescan', 2.0)
		self.cube_timeout = rospy.get_param('cube_timeout', 30.0)
		self.move_base_timeout = rospy.get_param('move_base_timeout', 30.0)
		self.axis_pan = rospy.get_param('axis_pan', 0.0)
		self.axis_tilt = rospy.get_param('axis_tilt', -0.3)
		self.axis_sweep_duration = rospy.get_param('axis_sweep_duration', 2.0)
		self.axis_sweep_angle = rospy.get_param('axis_sweep_angle', 0.3)


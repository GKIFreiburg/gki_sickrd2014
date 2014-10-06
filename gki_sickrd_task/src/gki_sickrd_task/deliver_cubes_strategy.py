#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import threading

from geometry_msgs.msg import PoseStamped

from gki_sickrd_task.params import Params
from gki_sickrd_task.actions import Actions
from gki_sickrd_task.percepts import Percepts, NotEnoughDataException
from gki_sickrd_task.tools import Tools

class DeliverCubesStrategy(object):
	def __init__(self):
		self.actions = Actions()
		self.percepts = Percepts()
		self.tools = Tools()
		self.previous_scan_pose = None
		self.decision_required = True		
		self.cube_loaded = False
		self.current_number = -1
		# align camera
		self.actions.look_to(self.look_done_cb)

	def decide(self, event):
		if not self.decision_required:
			return 
		try:
			Params.update()
			self.percepts.estimate_center_from_map()
			if not self.cube_loaded:
				try:
					# approach loading station
					self.approach_loading_station()
				except NotEnoughDataException as e:
					# not enough data to find loading station
					self.explore()
			else:
				try:
					# approach number
					self.approache_number()
				except NotEnoughDataException as e:
					# not enough data to find number
					self.explore()
		except Exception as e:
			rospy.logwarn('exception {}: {}'.format(type(e), e.message))
			rospy.sleep(1.0)

	def approach_loading_station(self):
		rospy.loginfo('looking for loading station...')
		loading_stations = self.percepts.get_loading_stations()
		current = self.tools.get_current_pose()
		loading_stations.sort(key=lambda object: self.tools.xy_point_distance(object.pose.pose.position, current.pose.position))
		stamped = PoseStamped()
		stamped.header = loading_stations[0].header
		stamped.pose = loading_stations[0].pose.pose
		if self.tools.xy_distance_to_robot(stamped) < 1.1 * Params.get().approach_distance:
			self.actions.approach(self.approach_loading_station_done_cb, self.approach_timeout_cb)
		else:
			approach = self.percepts.sample_approach_pose(stamped)
			self.actions.move_to(approach, self.move_base_cd, self.move_timeout_cb)
		self.decision_required = False

	def approache_number(self):
		rospy.loginfo('looking for loading number {}...'.format(self.current_number))
		number = self.percepts.get_number(self.current_number)
		stamped = PoseStamped()
		stamped.header = number.header
		stamped.pose = number.pose.pose
		if self.tools.xy_distance_to_robot(stamped) < 1.1 * Params.get().approach_distance:
			self.actions.approach(self.approach_number_done_cb, self.approach_timeout_cb)
		else:
			approach = self.percepts.sample_approach_pose(stamped)
			self.actions.move_to(approach, self.move_base_cd, self.move_timeout_cb)
		self.decision_required = False

	def explore(self):
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
		rospy.loginfo('exploring...')
		exploration_pose = self.percepts.sample_scan_pose()
		self.actions.move_to(exploration_pose, self.move_done_cb, self.move_timeout_cb)
		self.decision_required = False

	def sweep_done_cb(self, status, result):
		rospy.loginfo('camera_sweep: done')
		self.previous_scan_pose = self.tools.get_current_pose()
		self.actions.look_to(self.look_done_cb)

	def look_done_cb(self, status, result):
		rospy.loginfo('camera_look: done')
		self.decision_required = True

	def approach_loading_station_done_cb(self, status, result):
		rospy.loginfo('approach_loading_station: done')
		self.actions.cancel_move_timeout()
		self.percepts.wait_for_cube_sensor_change(done_cb)
		self.actions.enable_LEDs()
		# HACK: receive random cube
		self.cube_loaded = True
		self.current_number = self.tools.rnd.uniform[0, 9]
		self.decision_required = True

	def approach_number_done_cb(self, status, result):
		rospy.loginfo('approach_number: done')
		self.actions.cancel_move_timeout()
		# HACK: deliver cube
		self.cube_loaded = False
		self.decision_required = True

	def move_done_cb(self, status, result):
		rospy.loginfo('random_move: done')
		self.actions.cancel_move_timeout()
		self.decision_required = True

	def move_timeout_cb(self, event):
		rospy.loginfo('move: timeout')
		self.decision_required = True

	def approach_timeout_cb(self, event):
		rospy.loginfo('approach: timeout')
		# TODO: do something here... retreat?
		self.decision_required = True

	def cube_received(self, msg):
		pass


#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import sys,traceback

from geometry_msgs.msg import PoseStamped

from gki_sickrd_task.params import Params
from gki_sickrd_task.actions import Actions
from gki_sickrd_task.percepts import Percepts, NotEnoughDataException
from gki_sickrd_task.tools import Tools
from gki_sickrd_task.estop_guard import EstopGuard
from actionlib_msgs.msg import GoalStatus

class DeliverCubesStrategy(object):
	def __init__(self):
		self.actions = Actions()
		self.percepts = Percepts()
		self.tools = Tools()
		self.previous_scan_pose = None
		self.decision_required = True
		self.at_approach = False
		self.at_ring = False
		self.loading_cube = False
		self.unloading_cube = False
		self.cube_operation_failure = False
		self.status_string = ''
		EstopGuard.initialize(['/estop', '/joystick_estop'], self.estop_changed_cb)
		# align camera
		self.actions.look_to(self.look_done_cb)

	def print_state(self):
		rospy.loginfo('***')
		if self.cube_operation_failure:
			rospy.loginfo('cube operation failed, recovering...')
		elif self.percepts.cube_loaded():
			rospy.loginfo('cube loaded: {}'.format(self.percepts.current_number()))
		else:
			rospy.loginfo('no cube loaded.')
		rospy.loginfo(self.status_string)

	def decide(self, event):
		try:
			self.print_state()
			try:
				self.percepts.visualize_worldmodel()
			except NotEnoughDataException:
				pass
			if not self.decision_required:
				return 

			Params.update()
			self.percepts.estimate_center_from_map()

			# cube operation failure recovery
			if self.cube_operation_failure:
				if self.at_ring:
					self.retreat()
					return
				else:
					self.cube_operation_failure = False
					self.previous_scan_pose = None
					self.explore()
					return

			# cube operations
			if self.at_ring:
				if self.loading_cube:
					self.load_cube()
					return
				elif self.unloading_cube:
					self.unload_cube()
					return
				else:
					# cube operation complete
					self.retreat()
					return

			# drive and camera operations
			if not self.percepts.cube_loaded():
				try:
					# approach loading station
					self.approach_loading_station()
					return
				except NotEnoughDataException as e:
					# not enough data to find loading station
					self.explore()
					return
			else:
				try:
					# approach number
					self.approach_number()
					return
				except NotEnoughDataException as e:
					# not enough data to find number
					self.explore()
					return
		except NotEnoughDataException as e:
			rospy.logwarn('exception {}: {}'.format(type(e), e.message))
			rospy.sleep(1.0)
		except Exception as e:
			rospy.logerr(traceback.format_exc())

	def load_cube(self):
		self.actions.enable_LEDs()
		if not self.percepts.cube_loaded():
			self.status_string = 'waiting for cube...'
			rospy.loginfo(self.status_string)
			return # wait 
		number = self.percepts.current_number()
		self.status_string = 'cube received, number {}.'.format(number)
		rospy.loginfo(self.status_string)
		self.loading_cube = False
		self.actions.cancel_cube_timeout()

	def unload_cube(self):
		self.actions.enable_LEDs()
		if self.percepts.cube_loaded():
			self.status_string = 'waiting for cube removal...'
			rospy.loginfo(self.status_string)
			return # wait 
		self.status_string = 'cube removed.'
		rospy.loginfo(self.status_string)
		self.unloading_cube = False
		self.actions.cancel_cube_timeout()

	def approach_loading_station(self):
		loading_stations = self.percepts.get_loading_stations()
		current = self.tools.get_current_pose()
		loading_stations.sort(key=lambda object: self.tools.xy_point_distance(object.pose.pose.position, current.pose.position))
		stamped = PoseStamped()
		stamped.header = loading_stations[0].header
		stamped.pose = loading_stations[0].pose.pose
		if self.at_approach:
			self.status_string = 'approaching loading station...'
			rospy.loginfo(self.status_string)
			self.actions.approach(stamped, self.approach_loading_station_done_cb, self.approach_timeout_cb)
			self.at_approach = False
		else:
			self.status_string = 'moving to loading station...'
			rospy.loginfo(self.status_string)
			approach = self.percepts.sample_approach_pose(stamped)
			self.actions.move_to(approach, self.preapproach_done_cb, self.move_timeout_cb)
		self.decision_required = False

	def approach_number(self):
		number = self.percepts.current_number()
		banner = self.percepts.get_number_banner(number)
		stamped = PoseStamped()
		stamped.header = banner.header
		stamped.pose = banner.pose.pose
		if self.at_approach:
			self.status_string = 'approaching number {}...'.format(number)
			rospy.loginfo(self.status_string)
			self.actions.approach(stamped, self.approach_number_done_cb, self.approach_timeout_cb)
			self.at_approach = False
		else:
			self.status_string = 'moving to number {}...'.format(number)
			rospy.loginfo(self.status_string)
			approach = self.percepts.sample_approach_pose(stamped)
			self.actions.move_to(approach, self.preapproach_done_cb, self.move_timeout_cb)
		self.decision_required = False

	def retreat(self):
		self.status_string = 'retreating...'
		rospy.loginfo(self.status_string)
		self.actions.disable_LEDs()
		self.actions.retreat(self.retreat_done_cb, self.retreat_timeout_cb)
		self.decision_required = False

	def explore(self):
		# scan
		if not self.previous_scan_pose:
			self.previous_scan_pose = self.tools.get_current_pose()
		current_pose = self.tools.get_current_pose()
		if self.tools.xy_distance(self.previous_scan_pose, current_pose) > Params.get().min_travel_distance_for_rescan:
			self.status_string = 'sweeping...'
			rospy.loginfo(self.status_string)
			self.actions.camera_sweep(self.sweep_done_cb)
			self.decision_required = False
			return
		self.status_string = 'exploring...'
		rospy.loginfo(self.status_string)
		exploration_pose = self.percepts.sample_scan_pose()
		self.actions.move_to(exploration_pose, self.move_done_cb, self.move_timeout_cb)
		self.decision_required = False

	def sweep_done_cb(self, status, result):
		self.status_string = 'camera_sweep: done'
		rospy.loginfo(self.status_string)
		self.previous_scan_pose = self.tools.get_current_pose()
		self.actions.look_to(self.look_done_cb)
		self.decision_required = True

	def look_done_cb(self, status, result):
		self.status_string = 'camera_look: done'
		rospy.loginfo(self.status_string)
		self.decision_required = True

	def approach_loading_station_done_cb(self, status, result):
		if status == GoalStatus.SUCCEEDED:
			self.status_string = 'approach_loading_station: done'
			rospy.loginfo(self.status_string)
			self.at_ring = True
			self.loading_cube = True
			self.actions.enable_LEDs()
			self.actions.start_cube_operation_timer(self.cube_operation_timeout_cb)
		self.decision_required = True

	def approach_number_done_cb(self, status, result):
		if status == GoalStatus.SUCCEEDED:
			self.status_string = 'approach_number: done'
			rospy.loginfo(self.status_string)
			self.at_ring = True
			self.unloading_cube = True
			self.actions.start_cube_operation_timer(self.cube_operation_timeout_cb)
		self.decision_required = True

	def retreat_done_cb(self, status, result):
		self.status_string = 'retreat: done'
		rospy.loginfo(self.status_string)
		self.at_ring = False
		self.decision_required = True

	def preapproach_done_cb(self, status, result):
		if status == GoalStatus.SUCCEEDED:
			self.status_string = 'pre-approach: done'.format(status, result)
			rospy.loginfo(self.status_string)
			self.at_approach = True
		self.decision_required = True

	def move_done_cb(self, status, result):
		self.status_string = 'move: done'
		rospy.loginfo(self.status_string)
		self.decision_required = True

	def move_timeout_cb(self, event):
		self.status_string = 'move: timeout'
		rospy.loginfo(self.status_string)
		self.decision_required = True

	def approach_timeout_cb(self, event):
		self.status_string = 'approach: timeout'
		rospy.loginfo(self.status_string)
		# TODO: do something here... retreat?
		self.decision_required = True

	def retreat_timeout_cb(self, event):
		self.status_string = 'retreat: timeout'
		rospy.loginfo(self.status_string)
		# TODO: do something here... retreat?
		self.decision_required = True

	def cube_operation_timeout_cb(self, event):
		self.status_string = 'cube_operation: timeout'
		rospy.loginfo(self.status_string)
		self.cube_operation_failure = True
		self.decision_required = True

	def estop_changed_cb(self, stop):
		if stop:
			self.status_string = 'estop triggered.'
			rospy.loginfo(self.status_string)
			self.decision_required = False
			rospy.Rate(4).sleep() # let current decisions finish...
			self.actions.cancel_all_actions() # ... and then cancel them.
		else:
			self.status_string = 'estop released.'
			rospy.loginfo(self.status_string)
			self.decision_required = True

if __name__ == "__main__":
	rospy.init_node("sickrd_task")
	# strategy
	strategy = DeliverCubesStrategy()
	decision_timer = rospy.Timer(rospy.Duration(1.0), strategy.decide)
	rospy.spin()
	

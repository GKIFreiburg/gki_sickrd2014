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
from geometry_msgs.msg import Pose

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
		self.number_approach_failure = False
		self.standstill_failure = False
		self.scan_scheduled = False
		self.camera_aligned = False
		self.cube_status_string = ''
		self.status_string = ''
		EstopGuard.add_callback(self.estop_changed_cb)

	def print_state(self):
		rospy.loginfo('***')
		if self.cube_operation_failure:
			self.cube_status_string = 'recovery'
		elif self.percepts.cube_loaded():
			self.cube_status_string = 'cube: {}'.format(self.percepts.current_number())
		else:
			self.cube_status_string = 'no cube'
		rospy.loginfo(self.cube_status_string)
		rospy.loginfo(self.status_string)

	def decide(self, event):
		try:
			self.print_state()
			try:
				self.percepts.visualize_worldmodel('{}\n{}'.format(self.cube_status_string, self.status_string))
			except NotEnoughDataException:
				pass
			if not self.decision_required:
				return 

			Params().update()
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

			# number operation failure recovery
			if self.number_approach_failure:
				self.number_approach_failure = False
				self.previous_scan_pose = self.tools.get_current_pose()
				self.explore()
				return

			# no move failure recovery
			if self.standstill_failure:
				self.standstill_failure = False
				self.previous_scan_pose = self.tools.get_current_pose()
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
				
			if not self.camera_aligned:
				self.actions.look_to(self.look_ahead_done_cb)

			self.actions.start_standstill_timer(self.standstill_timeout_cb)
			# drive and camera operations
			if not self.percepts.cube_loaded():
				try:
					# approach loading station
					if Params().hack_octa_loading_station:
						self.approach_average_of_two_loading_station()
					else:
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
			self.actions.start_cube_operation_timer(self.cube_operation_timeout_cb)
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
			self.actions.start_cube_operation_timer(self.cube_operation_timeout_cb)
			self.status_string = 'waiting for cube removal...'
			rospy.loginfo(self.status_string)
			return # wait 
		self.status_string = 'cube removed.'
		rospy.loginfo(self.status_string)
		self.unloading_cube = False
		self.actions.cancel_cube_timeout()

	def approach_loading_station(self):
		current = self.tools.get_current_pose()
		loading_station = self.percepts.get_best_loading_station(current)
		if self.tools.is_good_approach_pose(current, loading_station):
			self.status_string = 'approaching loading station...'
			rospy.loginfo(self.status_string)
			stamped = PoseStamped()
			stamped.header = loading_station.header
			stamped.pose = self.tools.project_pose(loading_station.pose.pose)
			stamped.pose.position.z = -0.01
			self.actions.approach(stamped, self.approach_loading_station_done_cb, self.approach_timeout_cb)
		else:
			self.status_string = 'moving to loading station...'
			rospy.loginfo(self.status_string)
			approach = self.percepts.sample_approach_pose(loading_station.pose.pose, loading_station.header.frame_id)
			self.actions.move_to(approach, self.preapproach_done_cb, self.move_timeout_cb)
		self.decision_required = False

	def approach_average_of_two_loading_station(self):
		current = self.tools.get_current_pose()
		approach = self.percepts.get_best_average_approach_from_two_line_markers(current)
		if self.tools.xy_distance(current, approach) < 0.1:
			self.status_string = 'approaching loading station...'
			rospy.loginfo(self.status_string)
			stamped = PoseStamped(header=approach.header)
			offset = Pose()
			offset.position.x = Params().approach_distance
			offset.orientation.w = 1
			stamped.pose = self.tools.add_poses(approach.pose, offset)
			stamped.pose.position.z = -0.01
			self.actions.approach(stamped, self.approach_loading_station_done_cb, self.approach_timeout_cb)
		else:
			self.status_string = 'moving to loading station...'
			rospy.loginfo(self.status_string)
			self.actions.move_to(approach, self.preapproach_done_cb, self.move_timeout_cb)
		self.decision_required = False

	def approach_number(self):
		number = self.percepts.current_number()
		banner = self.percepts.get_number_banner(number)
		current = self.tools.get_current_pose()
		if banner.info.support < Params().min_trusted_support:
			# go to some good pose
			# look at the number for x seconds
			if self.tools.is_good_verification_pose(current, banner):
				# look at number
				self.status_string = 'looking at {}'.format(number)
				self.actions.look_at(PoseStamped(header=banner.header, pose = banner.pose.pose), done_cb=self.verification_lookat_done_cb)
			else:
				for i in range(20):
					stamped = self.tools.sample_verification_pose(banner, self.percepts.estimate_center_from_map())
					sampled_poses = []
					if self.percepts.check_path(current, stamped):
						self.status_string = 'moving to look at {}'.format(number)
						rospy.loginfo(self.status_string)
						self.actions.move_to(stamped, done_cb=self.move_to_verify_done_cb, timeout_cb=self.move_timeout_cb)
						self.decision_required = False
						return
					sampled_poses.append(stamped)
				self.tools.visualize_poses(sampled_poses, ns='failed_verification_poses')
				self.status_string = 'failure at {}'.format(number)
				self.number_approach_failure = True
				
		elif self.tools.is_good_approach_pose(current, banner):
			self.status_string = 'approaching number {}...'.format(number)
			rospy.loginfo(self.status_string)
			stamped = PoseStamped()
			stamped.header = banner.header
			stamped.pose = self.tools.project_pose(banner.pose.pose)
			#stamped.pose = self.tools.set_orientation_from_yaw(banner.pose.pose, self.tools.get_yaw(banner.pose.pose) + math.pi)
			stamped.pose.position.z = 0.01
			self.actions.approach(stamped, self.approach_number_done_cb, self.approach_timeout_cb)
		else:
			self.status_string = 'moving to number {}...'.format(number)
			rospy.loginfo(self.status_string)
			approach = self.percepts.sample_approach_pose(banner.pose.pose, banner.header.frame_id)
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
		if self.tools.xy_distance(self.previous_scan_pose, current_pose) > Params().min_travel_distance_for_rescan:
			self.status_string = 'sweeping...'
			rospy.loginfo(self.status_string)
			self.actions.camera_sweep(self.sweep_done_cb)
			self.decision_required = False
			return
		self.status_string = 'exploring...'
		rospy.loginfo(self.status_string)
		exploration_pose = self.percepts.sample_scan_pose()
		self.actions.move_to(exploration_pose, self.move_to_scan_done_cb, self.move_to_scan_timeout_cb)
		self.decision_required = False

	def sweep_done_cb(self, status, result):
		self.status_string = 'camera_sweep: done'
		rospy.loginfo(self.status_string)
		self.previous_scan_pose = self.tools.get_current_pose()
		self.camera_aligned = False
		self.decision_required = True

	def look_ahead_done_cb(self, status, result):
		self.status_string = 'camera_look_ahead: done'
		rospy.loginfo(self.status_string)
		self.camera_aligned = True
		self.decision_required = True

	def approach_loading_station_done_cb(self, status, result):
		if status == GoalStatus.SUCCEEDED:
			self.status_string = 'approach_loading_station: done'
			rospy.loginfo(self.status_string)
			self.loading_cube = True
			self.actions.enable_LEDs()
			#self.actions.start_cube_operation_timer(self.cube_operation_timeout_cb)
		self.at_ring = True
		self.decision_required = True

	def approach_number_done_cb(self, status, result):
		if status == GoalStatus.SUCCEEDED:
			self.status_string = 'approach_number: done'
			rospy.loginfo(self.status_string)
			self.unloading_cube = True
			#self.actions.start_cube_operation_timer(self.cube_operation_timeout_cb)
		self.at_ring = True
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
		self.decision_required = True

	def move_to_scan_done_cb(self, status, result):
		self.status_string = 'move to scan: done'
		rospy.loginfo(self.status_string)
		self.scan_scheduled = True
		self.decision_required = True

	def move_to_scan_timeout_cb(self, event):
		self.status_string = 'move to scan: timeout'
		rospy.loginfo(self.status_string)
		self.decision_required = True

	def move_to_verify_done_cb(self, status, result):
		self.status_string = 'move to verify: done'
		rospy.loginfo(self.status_string)
		self.decision_required = True

	def move_timeout_cb(self, event):
		self.status_string = 'move to verify: timeout'
		rospy.loginfo(self.status_string)
		self.decision_required = True

	def approach_timeout_cb(self, event):
		self.status_string = 'approach: timeout'
		rospy.loginfo(self.status_string)
		self.at_ring = True
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

	def verification_lookat_done_cb(self, status, result):
		self.actions.start_verification_timer(self.verification_timeout_cb)

	def verification_timeout_cb(self, event):
		self.status_string = 'verification: done'
		rospy.loginfo(self.status_string)
		self.decision_required = True

	def standstill_timeout_cb(self, event):
		self.status_string = 'no_move: timeout'
		rospy.loginfo(self.status_string)
		self.standstill_failure = True
		self.decision_required = True

	def estop_changed_cb(self, stop):
		if stop:
			self.status_string = 'estop triggered.'
			rospy.loginfo(self.status_string)
			self.decision_required = False
			rospy.Rate(4).sleep() # let current decisions finish...
			self.actions.cancel_all_actions() # ... and then cancel them.
			self.actions.cancel_standstill_timer()
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
	

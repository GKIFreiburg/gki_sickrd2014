#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import random
import copy

import tf
from visualization_msgs.msg import Marker, MarkerArray
from kobuki_msgs.msg import Led
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from camera_control_msgs.msg import CameraAction, CameraGoal, CameraResult
from geometry_msgs.msg import PoseStamped
from hector_worldmodel_msgs.msg import ObjectModel, Object, ObjectInfo
from gki_sickrd_task.params import Params
from gki_sickrd_task.tools import Tools
from actionlib_msgs.msg import GoalStatus

class ActionWrapper(object):
	def __init__(self, action_client, goal, done_cb, timeout_cb=None, timeout=None):
		self._done_cb = done_cb
		self._timeout_cb = None
		self._timer = None
		if timeout_cb and timeout:
			self._timeout_cb = timeout_cb
			self._timer = rospy.Timer(rospy.Duration(timeout), self.timeout_cb, oneshot=True)
		self._action_client = action_client
		self._action_client.send_goal(goal, done_cb=self.done_cb)

	def is_active(self):
		if self._action_client.get_state() == GoalStatus.ACTIVE:
			return True
		return False

	def cancel(self):
		self._action_client.cancel_all_goals()
		if self._timer:
			self._timer.shutdown()

	def done_cb(self, status, result):
		if self._timer:
			if self._timer.is_alive():
				self._timer.shutdown()
		self._done_cb(status, result)

	def timeout_cb(self, event):
		self._action_client.cancel_all_goals()
		self._timeout_cb(event)

class Actions(object):
	def __init__(self):
		self.tools = Tools()
		self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
		self.approach_client = SimpleActionClient('/approach_action', MoveBaseAction)
		self.retreat_client = SimpleActionClient('/retreat_action', MoveBaseAction)
		self.camera_ptz_client = SimpleActionClient('/axis/axis_control', CameraAction)
		self.led_publishers = [rospy.Publisher('/led0', Led), rospy.Publisher('/led1', Led), rospy.Publisher('/led2', Led)]
		self.verification_timer = None
		self.cube_timer = None
		self.standstill_timer = None
		self._action = None

		for client in [self.move_base_client, self.approach_client, self.retreat_client, self.camera_ptz_client]:
			if client:
				rospy.loginfo('waiting for {} action server...'.format(client.action_client.ns))
				client.wait_for_server()
				rospy.loginfo('connected to {} action server'.format(client.action_client.ns))
		rospy.loginfo('actions initialized')

	def cancel_all_actions(self):
		#rospy.loginfo('canceling all actions...')
		if self._action:
			if self._action.is_active():
				self._action.cancel()
		self.cancel_cube_timer()
		self.cancel_verification_timer()
		self.disable_LEDs()

	def random_move(self, done_cb, timeout_cb):
		stamped = PoseStamped()
		range = self.tools.rnd.uniform(1.0, 2.5)
		yaw_offset = self.tools.rnd.uniform(-0.15*math.pi, 0.35*math.pi)
		stamped.pose.position.x = range * math.cos(yaw_offset)
		stamped.pose.position.y = range * math.sin(yaw_offset)
		quat = tf.transformations.quaternion_from_euler(0, 0, yaw_offset)
		stamped.pose.orientation.x = quat[0]
		stamped.pose.orientation.y = quat[1]
		stamped.pose.orientation.z = quat[2]
		stamped.pose.orientation.w = quat[3]
		stamped.header.frame_id = 'base_footprint'
		stamped.header.stamp = rospy.Time.now()
		self.cancel_standstill_timer()
		self.move_to(stamped, done_cb, timeout_cb)

	def move_to(self, stamped, done_cb, timeout_cb):
		goal = MoveBaseGoal()
		goal.target_pose = stamped
		goal.target_pose.header.stamp = rospy.Time(0)
		msg = MarkerArray()
		msg.markers.append(self.tools.create_pose_marker(stamped))
		self.tools.visualization_publisher.publish(msg)
		self.cancel_all_actions()
		self.cancel_standstill_timer()
		self._action = ActionWrapper(done_cb=done_cb, timeout_cb=timeout_cb, timeout=Params().move_base_timeout, action_client=self.move_base_client, goal=goal)

	def approach(self, stamped, done_cb, timeout_cb):
		goal = MoveBaseGoal()
		goal.target_pose = stamped
		goal.target_pose.header.stamp = rospy.Time(0)
		msg = MarkerArray()
		msg.markers.append(self.tools.create_pose_marker(goal.target_pose))
		msg.markers[-1].color.r = 0.5
		msg.markers[-1].color.g = 0.8
		self.tools.visualization_publisher.publish(msg)
		self.cancel_all_actions()
		self.cancel_standstill_timer()
		self._action = ActionWrapper(done_cb=done_cb, timeout_cb=timeout_cb, timeout=Params().approach_timeout, action_client=self.approach_client, goal=goal)

	def retreat(self, done_cb, timeout_cb):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'base_footprint'
		goal.target_pose.pose.position.x = -(Params().approach_distance - Params().ring_distance)
		goal.target_pose.header.stamp = rospy.Time(0)
		goal.target_pose.pose.orientation.w = 1
		goal.target_pose = self.tools.transform_pose('map', goal.target_pose)
		msg = MarkerArray()
		msg.markers.append(self.tools.create_pose_marker(goal.target_pose))
		msg.markers[-1].color.r = 0.8
		msg.markers[-1].color.g = 0.5
		self.tools.visualization_publisher.publish(msg)
		self.cancel_all_actions()
		self.cancel_standstill_timer()
		self._action = ActionWrapper(done_cb=done_cb, timeout_cb=timeout_cb, timeout=Params().approach_timeout, action_client=self.retreat_client, goal=goal)

	def camera_sweep(self, done_cb, delta_yaw=None, duration=None, zoom=1):
		if not delta_yaw:
			delta_yaw = Params().axis_sweep_angle
		if not duration:
			duration = Params().axis_sweep_duration
		goal = CameraGoal()
		goal.command = 2 #sweep
		goal.tilt = Params().axis_tilt
		goal.sweep_step = delta_yaw
		goal.sweep_hold_time = duration
		self.cancel_all_actions()
		self._action = ActionWrapper(action_client=self.camera_ptz_client, goal=goal, done_cb=done_cb)

	def look_at(self, stamped, done_cb):
		stamped.header.stamp = rospy.Time(0)
		ps = Tools.tf_listener.transformPose('axis_link', stamped)
		yaw = math.atan2(ps.pose.position.y, ps.pose.position.x)
		pitch = Params().axis_tilt #math.atan2(ps.pose.position.z, ps.pose.position.x)
		self.look_to(done_cb, yaw, pitch)

	def look_to(self, done_cb, yaw=None, pitch=None, zoom=1):
		if not yaw:
			yaw = Params().axis_pan
		if not pitch:
			pitch = Params().axis_tilt
		goal = CameraGoal()
		goal.command = 1
		goal.pan = yaw
		goal.tilt = pitch
		self.cancel_all_actions()
		self._action = ActionWrapper(action_client=self.camera_ptz_client, goal=goal, done_cb=done_cb)

	def start_verification_timer(self, timeout_cb):
		self.verification_timer = rospy.Timer(rospy.Duration(Params().verification_duration), timeout_cb, oneshot=True)

	def cancel_verification_timer(self):
		if not self.cube_timer:
			return 
		if self.cube_timer.is_alive():
			self.cube_timer.shutdown()

	def start_cube_operation_timer(self, timeout_cb):
		if self.cube_timer:
			if self.cube_timer.is_alive():
				return
		self.cube_timer = rospy.Timer(rospy.Duration(Params().cube_timeout), timeout_cb, oneshot=True)

	def start_standstill_timer(self, timeout_cb):
		if self.standstill_timer:
			if self.standstill_timer.is_alive():
				return
		self.standstill_timer = rospy.Timer(rospy.Duration(Params().standstill_timeout), timeout_cb, oneshot=True)

	def cancel_standstill_timer(self):
		if not self.standstill_timer:
			return 
		if self.standstill_timer.is_alive():
			self.standstill_timer.shutdown()

	def cancel_cube_timer(self):
		if not self.cube_timer:
			return 
		if self.cube_timer.is_alive():
			self.cube_timer.shutdown()

	def enable_LEDs(self):
		msg = Led()
		msg.value = Led.GREEN
		for pub in self.led_publishers:
			pub.publish(msg)

	def disable_LEDs(self):
		msg = Led()
		msg.value = Led.BLACK
		for pub in self.led_publishers:
			pub.publish(msg)

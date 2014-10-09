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

class Actions(object):
	def __init__(self):
		self.tools = Tools()
		self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
		self.approach_client = SimpleActionClient('/approach_action', MoveBaseAction)
		self.retreat_client = SimpleActionClient('/retreat_action', MoveBaseAction)
		self.camera_ptz_client = SimpleActionClient('/axis/axis_control', CameraAction)
		self.led_publishers = [rospy.Publisher('/led0', Led), rospy.Publisher('/led1', Led), rospy.Publisher('/led2', Led)]
		self.approach_timeout_timer = None
		self.move_timeout_timer = None
		self.cube_timeout_timer = None

		for client in [self.move_base_client, self.approach_client, self.retreat_client, self.camera_ptz_client]:
			if client:
				rospy.loginfo('waiting for {} action server...'.format(client.action_client.ns))
				client.wait_for_server()
				rospy.loginfo('connected to {} action server'.format(client.action_client.ns))
		rospy.loginfo('actions initialized')

	def cancel_all_actions(self):
		rospy.loginfo('canceling all actions...')
		for client in [self.move_base_client, self.approach_client, self.camera_ptz_client]:
			client.cancel_all_goals()
		for timer in [self.approach_timeout_timer, self.move_timeout_timer, self.cube_timeout_timer]:
			if timer:
				timer.shutdown()
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
		self.move_to(stamped, done_cb, timeout_cb)

	def stop(self):
		self.cancel_move_timeout()
		self.move_base_client.cancel_all_goals()

	def move_to(self, stamped, done_cb, timeout_cb):
		goal = MoveBaseGoal()
		goal.target_pose = stamped
		msg = MarkerArray()
		msg.markers.append(self.tools.create_pose_marker(stamped))
		self.tools.visualization_publisher.publish(msg)
		self.move_timeout_timer = rospy.Timer(rospy.Duration(Params.get().move_base_timeout), timeout_cb, oneshot=True)
		self.move_base_client.send_goal(goal, done_cb=done_cb)

	def approach(self, done_cb, timeout_cb):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'base_footprint'
		goal.target_pose.pose.position.x = Params.get().approach_distance - Params.get().ring_distance
		goal.target_pose.pose.orientation.w = 1
		goal.target_pose = self.tools.transform_pose('map', goal.target_pose)
		msg = MarkerArray()
		msg.markers.append(self.tools.create_pose_marker(goal.target_pose))
		msg.markers[-1].color.r = 0.5
		msg.markers[-1].color.g = 0.8
		self.tools.visualization_publisher.publish(msg)
		self.move_timeout_timer = rospy.Timer(rospy.Duration(Params.get().move_base_timeout), timeout_cb, oneshot=True)
		self.approach_client.send_goal(goal, done_cb=done_cb)

	def retreat(self, done_cb, timeout_cb):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'base_footprint'
		goal.target_pose.pose.position.x = -(Params.get().approach_distance - Params.get().ring_distance)
		goal.target_pose.pose.orientation.w = 1
		goal.target_pose = self.tools.transform_pose('map', goal.target_pose)
		msg = MarkerArray()
		msg.markers.append(self.tools.create_pose_marker(goal.target_pose))
		msg.markers[-1].color.r = 0.8
		msg.markers[-1].color.g = 0.5
		self.tools.visualization_publisher.publish(msg)
		self.move_timeout_timer = rospy.Timer(rospy.Duration(Params.get().move_base_timeout), timeout_cb, oneshot=True)
		self.retreat_client.send_goal(goal, done_cb=done_cb)

	def cancel_move_timeout(self):
		if not self.move_timeout_timer:
			return 
		if self.move_timeout_timer.is_alive():
			self.move_timeout_timer.shutdown()

	def look_at(self, stamped, done_cb):
		ps = Tools.tf_listener.transformPose('axis_link', stamped)
		yaw = math.atan2(ps.pose.position.y, ps.pose.position.x)
		pitch = math.atan2(ps.pose.position.z, ps.pose.position.x)
		self.look_to(done_cb, yaw, pitch)

	def camera_sweep(self, done_cb, delta_yaw=Params.get().axis_sweep_angle, duration=Params.get().axis_sweep_duration, zoom=1):
		goal = CameraGoal()
		goal.command = 2 #sweep
		goal.tilt = Params.get().axis_tilt
		goal.sweep_step = delta_yaw
		goal.sweep_hold_time = duration
		self.camera_ptz_client.send_goal(goal, done_cb=done_cb)

	def look_to(self, done_cb, yaw=Params.get().axis_pan, pitch=Params.get().axis_tilt, zoom=1):
		goal = CameraGoal()
		goal.command = 1
		goal.pan = yaw
		goal.tilt = pitch
		self.camera_ptz_client.send_goal(goal, done_cb=done_cb)

	def start_cube_operation_timer(self, timeout_cb):
		self.cube_timeout_timer = rospy.Timer(rospy.Duration(Params.get().cube_timeout), timeout_cb, oneshot=True)

	def cancel_cube_timeout(self):
		if not self.cube_timeout_timer:
			return 
		if self.cube_timeout_timer.is_alive():
			self.cube_timeout_timer.shutdown()

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

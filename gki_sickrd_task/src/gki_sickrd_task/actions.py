#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import random
import copy

import tf
from visualization_msgs.msg import Marker, MarkerArray
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from camera_control_msgs.msg import CameraAction, CameraGoal, CameraResult
from geometry_msgs.msg import PoseStamped
from hector_worldmodel_msgs.msg import ObjectModel, Object, ObjectInfo
from gki_sickrd_task.tools import Tools

class Actions(object):
	def __init__(self):
		self.tools = Tools()
		self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
		self.camera_ptz_client = SimpleActionClient('/axis/axis_control', CameraAction)
		self.led_client = None
		self.move_timeout_timer = None #rospy.Timer(rospy.Duration(0.5), None)
		
		self.move_base_timeout = rospy.get_param('move_base_timeout', 30.0)
		
		for client in [self.move_base_client, self.camera_ptz_client, self.led_client]:
			if client:
				rospy.loginfo('waiting for {} action server...'.format(client.action_client.ns))
				client.wait_for_server()
				rospy.loginfo('connected to {} action server'.format(client.action_client.ns))
		rospy.loginfo('actions initialized')
		
	def create_pose_marker(self, stamped):
		marker = Marker()
		marker.type = Marker.ARROW
		marker.action = Marker.ADD
		marker.color.a = 0.8
		marker.color.r = 0.8
		marker.color.g = 0.2
		marker.color.b = 0.2
		marker.scale.x = 0.2
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.pose = copy.deepcopy(stamped.pose)
		marker.pose.position.z += 0.2
		marker.header = stamped.header
		marker.ns = 'actions'
		return marker
	
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
		self.move_timeout_timer = rospy.Timer(rospy.Duration(self.move_base_timeout), timeout_cb, oneshot=True)
		self.move_base_client.send_goal(goal, done_cb=done_cb)
	
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
	
	def camera_sweep(self, done_cb, delta_yaw=0.3, duration=0.5, zoom=1):
		goal = CameraGoal()
		goal.command = 2 #sweep
		goal.sweep_step = delta_yaw
		goal.sweep_hold_time = duration
		self.camera_ptz_client.send_goal(goal, done_cb=done_cb)
	
	def look_to(self, done_cb, yaw=0.0, pitch=-0.1, zoom=1):
		goal = CameraGoal()
		goal.command = 1
		goal.pan = yaw
		goal.tilt = pitch
		self.camera_ptz_client.send_goal(goal, done_cb=done_cb)
	
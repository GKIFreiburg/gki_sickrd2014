#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math

import tf
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from camera_control_msgs.msg import CameraAction, CameraGoal, CameraResult

class Actions(object):
	def __init__(self):
		self.tf_listener = tf.TransformListener()
		self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
		self.camera_ptz_client = SimpleActionClient('/axis/axis_control', CameraAction)
		self.led_client = None
		
		for client in [self.move_base_client, self.camera_ptz_client, self.led_client]:
			if client:
				rospy.loginfo('waiting for {} action server...'.format(client.action_client.ns))
				client.wait_for_server()
				rospy.loginfo('connected to {} action server'.format(client.action_client.ns))
		rospy.loginfo('actions initialized')
		
	def move_to(self, stamped):
		pass
	
	def explore(self):
		pass
	
	def look_at(self, stamped):
		pass
	
	def camera_sweep(self, delta_yaw=0.3, duration=0.5, zoom=1):
		pass
	
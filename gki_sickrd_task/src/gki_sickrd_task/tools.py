#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import random
import copy

import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

class Tools(object):
	tf_listener = None
	rnd = None
	visualization_publisher = None
	def __init__(self):
		if Tools.tf_listener == None:
			Tools.tf_listener = tf.TransformListener()
		self.tf_listener = Tools.tf_listener
		if Tools.rnd == None:
			Tools.rnd = random.Random()
		self.rnd = Tools.rnd
		# visualization
		if Tools.visualization_publisher == None:
			Tools.visualization_publisher = rospy.Publisher('/task/visualization_marker_array', MarkerArray, latch=True)
		self.visualization_publisher = Tools.visualization_publisher
		
	def create_pose_marker(self, stamped, ns='', id=0, z_offset=0.0):
		marker = Marker()
		marker.type = Marker.ARROW
		marker.action = Marker.ADD
		marker.color.a = 0.8
		marker.color.r = 0.2
		marker.color.g = 0.2
		marker.color.b = 0.2
		marker.scale.x = 0.2
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.pose = copy.deepcopy(stamped.pose)
		marker.pose.position.z += z_offset
		marker.header = stamped.header
		marker.ns = ns
		marker.id = id
		return marker
	
	def get_current_pose(self, global_frame='map'):
		pose = PoseStamped()
		pose.pose.orientation.w = 1
		pose.header.frame_id = 'base_footprint'
		return self.tf_listener.transformPose(target_frame=global_frame, ps=pose)
	
	def xy_distance(self, ps1, ps2):
		if ps1.header.frame_id != ps2.header.frame_id:
			ps2 = self.tf_listener.transformPose(target_frame=ps1.header.frame_id, ps=ps2)
		return math.hypot(ps1.pose.position.x-ps2.pose.position.x, ps1.pose.position.y-ps2.pose.position.y)
	
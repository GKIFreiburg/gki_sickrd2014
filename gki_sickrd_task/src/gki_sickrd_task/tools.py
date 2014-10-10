#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import random
import copy

import tf
import tf_conversions.posemath as pm
import PyKDL
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point

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
	
	def create_delete_marker(self, ns, id):
		marker = Marker()
		marker.header.frame_id = 'map'
		marker.pose.orientation.w = 1
		marker.action = Marker.DELETE
		marker.id = id
		return marker
		
	def create_status_marker(self, text):
		marker = Marker()
		marker.header.frame_id = 'base_footprint'
		marker.pose.position.z += 1
		marker.pose.orientation.w = 1
		marker.type = Marker.TEXT_VIEW_FACING
		marker.text = text
		marker.frame_locked = True
		marker.ns = 'status'
		marker.scale.z = 0.3
		marker.color.r = 0.8
		marker.color.g = 0.5
		marker.color.b = 0.1
		marker.color.a = 1.0
		return marker
		
	def create_loading_station_markers(self, pose_percept=None, approach_pose=None, id=0):
		if pose_percept:
			station = Marker()
			station.header.frame_id = pose_percept.header.frame_id
			station.pose = copy.deepcopy(pose_percept.pose.pose)
			station.ns = 'loading_stations'
			station.id = id
			station.color.r = 0.4
			station.color.g = 0.4
			station.color.b = 0.8
			station.color.a = 1.0
			station.type = Marker.CUBE
			station.scale.x = 0.02
			station.scale.y = 0.2
			station.scale.z = 0.2
			if approach_pose:
				approach = copy.deepcopy(station)
				approach.ns = 'loading_approach'
				approach.type = Marker.ARROW
				approach.pose = copy.deepcopy(approach_pose.pose)
				approach.scale = Point(0.2, 0.1, 0.1)
			else:
				approach = self.create_delete_marker('loading_approach', id)
			return station, approach
		return self.create_delete_marker('loading_stations', id), self.create_delete_marker('loading_approach', id)

	def create_banner_markers(self, pose_percept=None, approach_pose=None, id=0):
		if pose_percept:
			banner = Marker()
			banner.header.frame_id = pose_percept.header.frame_id
			banner.pose = copy.deepcopy(pose_percept.pose.pose)
			banner.ns = 'number_banners'
			banner.id = id
			i = id + 4
			banner.color.r = i/9*0.4
			banner.color.g = i%9/3*0.4+0.1
			banner.color.b = i%3*0.4+0.2
			banner.color.a = 1.0
			banner.type = Marker.CUBE
			banner.scale.x = 0.02
			banner.scale.y = 0.2
			banner.scale.z = 0.2
			label = copy.deepcopy(banner)
			label.type = Marker.TEXT_VIEW_FACING
			label.text = '{} ({:3.1f})'.format(id, pose_percept.info.support)
			label.ns = 'banner_label'
			label.pose.position.z += 0.4
			label.scale.z = 0.4
			if approach_pose:
				approach = copy.deepcopy(banner)
				approach.ns = 'banner_approach'
				approach.type = Marker.ARROW
				approach.pose = copy.deepcopy(approach_pose.pose)
				approach.scale = Point(0.2, 0.1, 0.1)
			else:
				approach = self.create_delete_marker('banner_approach', id)
			return banner, label, approach
		return self.create_delete_marker('number_banners', id), self.create_delete_marker('banner_label', id), self.create_delete_marker('banner_approach', id)

	def get_current_pose(self, frame_id='map'):
		pose = PoseStamped()
		pose.pose.orientation.w = 1
		pose.header.frame_id = 'base_footprint'
		return self.tf_listener.transformPose(target_frame=frame_id, ps=pose)
	
	def transform_pose(self, frame_id, stamped):
		return self.tf_listener.transformPose(target_frame=frame_id, ps=stamped)
	
	def add_poses(self, pose1, pose2):
		transform1 = pm.fromMsg(pose1)
		transform2 = pm.fromMsg(pose2)
		sum = transform1 * transform2
		return pm.toMsg(sum)
	
	def xy_distance_to_robot(self, ps2):
		ps1 = self.get_current_pose(ps2.header.frame_id)
		return self.xy_point_distance(ps1.pose.position, ps2.pose.position)

	def xy_distance(self, ps1, ps2):
		if ps1.header.frame_id != ps2.header.frame_id:
			ps2 = self.transform_pose(frame_id=ps1.header.frame_id, stamped=ps2)
		return self.xy_point_distance(ps1.pose.position, ps2.pose.position)

	def xy_point_distance(self, p1, p2):
		return math.hypot(p1.x-p2.x, p1.y-p2.y)
	
	def get_yaw(self, pose):
		frame = pm.fromMsg(pose)
		[roll, pitch, yaw] = frame.M.GetRPY()
		return yaw
	
	def project_pose(self, pose):
		frame = pm.fromMsg(pose)
		[roll, pitch, yaw] = frame.M.GetRPY()
		frame.M = PyKDL.Rotation.RPY(0, 0, yaw)
		projected = pm.toMsg(frame)
		projected.position.z = 0
		return projected

	def set_orientation_from_yaw(self, pose, yaw):
		frame = pm.fromMsg(pose)
		frame.M = PyKDL.Rotation.RPY(0, 0, yaw)
		return pm.toMsg(frame)

	def sample_verification_pose(self, banner):
		pass
	

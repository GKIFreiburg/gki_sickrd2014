#!/usr/bin/env python

import roslib; roslib.load_manifest("stage_sickrd")
import rospy
import sys, traceback, math, tf, copy
import numpy as np
import random
import yaml
from genpy.message import fill_message_args
from gki_pose_creator.msg import NamedPoseStampedList

from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from hector_worldmodel_msgs.msg import PosePercept, PerceptInfo
from stage_sickrd.camera_model import CameraModel


def create_camera_marker(camera):
	m = Marker()
	m.action = Marker.ADD
	m.type = Marker.LINE_LIST
	m.color.a = 0.9
	m.color.r = 0.5
	m.color.g = 0.7
	m.color.b = 0.5
	m.header.frame_id = camera.frame_id
	m.frame_locked = True
	m.ns = camera.frame_id
	m.pose.orientation.w = 1
	m.scale.x = 0.02
	near_x = camera.min_range * math.tan(camera.max_yaw)
	far_x = camera.max_range * math.tan(camera.max_yaw)
	near_y = camera.min_range * math.tan(camera.max_pitch)
	far_y = camera.max_range * math.tan(camera.max_pitch)
	
	m.points.append(Point(x=near_x, y=near_y, z=camera.min_range))
	m.points.append(Point(x=-near_x, y=near_y, z=camera.min_range))
	m.points.append(Point(x=near_x, y=-near_y, z=camera.min_range))
	m.points.append(Point(x=-near_x, y=-near_y, z=camera.min_range))
	m.points.append(Point(x=near_x, y=near_y, z=camera.min_range))
	m.points.append(Point(x=near_x, y=-near_y, z=camera.min_range))
	m.points.append(Point(x=-near_x, y=near_y, z=camera.min_range))
	m.points.append(Point(x=-near_x, y=-near_y, z=camera.min_range))
	
	m.points.append(Point(x=far_x, y=far_y, z=camera.max_range))
	m.points.append(Point(x=-far_x, y=far_y, z=camera.max_range))
	m.points.append(Point(x=far_x, y=-far_y, z=camera.max_range))
	m.points.append(Point(x=-far_x, y=-far_y, z=camera.max_range))
	m.points.append(Point(x=far_x, y=far_y, z=camera.max_range))
	m.points.append(Point(x=far_x, y=-far_y, z=camera.max_range))
	m.points.append(Point(x=-far_x, y=far_y, z=camera.max_range))
	m.points.append(Point(x=-far_x, y=-far_y, z=camera.max_range))
	
	m.points.append(Point(x=near_x, y=near_y, z=camera.min_range))
	m.points.append(Point(x=far_x, y=far_y, z=camera.max_range))
	m.points.append(Point(x=-near_x, y=near_y, z=camera.min_range))
	m.points.append(Point(x=-far_x, y=far_y, z=camera.max_range))
	m.points.append(Point(x=near_x, y=-near_y, z=camera.min_range))
	m.points.append(Point(x=far_x, y=-far_y, z=camera.max_range))
	m.points.append(Point(x=-near_x, y=-near_y, z=camera.min_range))
	m.points.append(Point(x=-far_x, y=-far_y, z=camera.max_range))
	return m

def load_pose_percepts(filename):
	poses = NamedPoseStampedList()
	pose_percepts = []
	with open(filename, 'r') as file:
		content = yaml.load(file.read())
		msg = NamedPoseStampedList()
		fill_message_args(msg, [content])
		for pose in msg.poses:
			pose_percept = PosePercept()
			pose_percept.header.frame_id = pose.stamped.header.frame_id
			pose_percept.pose.pose = pose.stamped.pose
			pose_percepts.append(pose_percept)
	return pose_percepts

def create_number_banner_marker(percept, id):
	marker = Marker()
	marker.pose = percept.pose.pose
	marker.header.frame_id = percept.header.frame_id
	marker.action = Marker.ADD
	marker.type = Marker.CUBE
	marker.id = id
	marker.ns = 'banner'
	marker.color.r = 0.3
	marker.color.g = 0.3
	marker.color.b = 0.8
	marker.color.a = 0.8
	marker.scale.x = 0.02
	marker.scale.y = 0.4
	marker.scale.z = 0.6
	return marker

def create_target_marker(percept, id):
	marker = Marker()
	marker.pose = percept.pose.pose
	marker.header.frame_id = percept.header.frame_id
	marker.action = Marker.ADD
	marker.type = Marker.CUBE
	marker.id = id
	marker.ns = 'target'
	marker.color.r = 0.3
	marker.color.g = 0.3
	marker.color.b = 0.8
	marker.color.a = 0.8
	marker.scale.x = 0.02
	marker.scale.y = 0.2
	marker.scale.z = 0.2
	return marker

def xy_distance(point1, point2):
	return math.hypot(point1.x - point2.x, point1.y - point2.y)

if __name__ == "__main__":
	rospy.init_node("percept_simulation", log_level=rospy.INFO)
	tf_listener = tf.TransformListener()
	percept_publisher = rospy.Publisher('/worldmodel/pose_percept', PosePercept)
	visualization_publisher = rospy.Publisher('/simluation/visualization_marker_array', MarkerArray, latch=True)
	axis_camera = CameraModel(tf_listener=tf_listener, frame_id='axis_optical_link')
	#logitech_hd = CameraModel(tf_listener=tf_listener, frame_id='logitech_optical_link')
	cameras = [axis_camera]
	
	# load numbers and taget markers
	number_banner_file = rospy.get_param('~number_banner_file')
	pose_percepts = load_pose_percepts(number_banner_file)
	banners = []
	i = 0
	random.seed(rospy.Time.now())
	random.shuffle(pose_percepts)
	for percept in pose_percepts:
		percept.info.class_id = 'number_banner_{}'.format(len(banners))
		percept.info.class_support = 0.75
		percept.info.object_id = ''
		percept.info.object_support = 0
		percept.info.name = ''
		banners.append(percept)
		if len(banners) >= 10:
			break

	target_info = PerceptInfo()
	target_info.class_id = 'target_marker'
	target_info.class_support = 0.75
	target_info.object_id = ''
	target_info.object_support = 0
	target_info.name = ''
	target_marker_file = rospy.get_param('~target_marker_file')
	pose_percepts = load_pose_percepts(target_marker_file)
	target_markers = []
	for percept in pose_percepts:
		for banner in banners:
			if xy_distance(percept.pose.pose.position, banner.pose.pose.position) > 0.01:
				continue
			percept.info = target_info
			target_markers.append(percept)
			break
	target_marker_file = rospy.get_param('~loading_station_file')
	pose_percepts = load_pose_percepts(target_marker_file)
	for percept in pose_percepts:
		percept.info = target_info
		target_markers.append(percept)
	
	msg = MarkerArray()
	for percept in banners:
		msg.markers.append(create_number_banner_marker(percept, len(msg.markers)))
	for percept in target_markers:
		msg.markers.append(create_target_marker(percept, len(msg.markers)))
	for camera in cameras:
		msg.markers.append(create_camera_marker(camera))
	visualization_publisher.publish(msg)
	
	percepts = []
	percepts.extend(banners)
	percepts.extend(target_markers)
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		perceived = []
		stamp=rospy.Time.now()
		for camera in cameras:
			perceived.extend(camera.perceive(pose_percept_list=percepts, stamp=stamp))
		rospy.loginfo('cameras perceived {} objects.'.format(len(perceived)))
		for percept in perceived:
			percept_publisher.publish(percept)
		rate.sleep()
	


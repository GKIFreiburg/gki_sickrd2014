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
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from nav_msgs.msg import OccupancyGrid
from hector_worldmodel_msgs.msg import ObjectModel, Object, ObjectInfo

class NumberNotFoundException(Exception):
	def __init__(self, number):
		self.message = 'no instance of banner with value {} are known.'.format(number)
class NotEnoughDataException(Exception):
	def __init__(self, message):
		self.message = message


class Worldmodel(object):
	def __init__(self, tf_listener, viz_pub):
		self.tf_listener = tf_listener
		self.rnd = random.Random()
		self.optimal_exploration_distance = rospy.get_param('optimal_exploration_distance', 3.0)
		self.model = None
		self.numbers = {}
		self.targets = {}
		self.worldmodel = None
		self.worldmodel_subscriber = rospy.Subscriber('/worldmodel/objects', ObjectModel, self.worldmodel_cb)
		self.map = None
		self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
		
		# visualization
		self.visualization_publisher = viz_pub
		rospy.loginfo('worldmodel initialized')
		
	def create_pose_marker(self, stamped):
		marker = Marker()
		marker.type = Marker.ARROW
		marker.action = Marker.ADD
		marker.color.a = 0.8
		marker.color.r = 0.2
		marker.color.g = 0.2
		marker.color.b = 0.8
		marker.scale.x = 0.2
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.pose = copy.deepcopy(stamped.pose)
		marker.pose.position.z += 0.2
		marker.header = stamped.header
		marker.ns = 'worldmodel'
		return marker
	
	def worldmodel_cb(self, msg):
		rospy.loginfo('new model data')
		self.model = msg
# 		for object in self.model.objects:
# 			if object.info.class_id == 'target_marker':
# 				self.targets[object.info.object_id] = object
# 			else:
# 				if object.info.class_id not in self.numbers:
# 					self.numbers[object.info.class_id] = {}
# 				self.numbers[object.info.class_id][object.info.object_id] = object
				
	def map_cb(self, msg):
		rospy.loginfo('new map data')
		self.map = msg
	
	def xy_distance(self, ps1, ps2):
		if ps1.header.frame_id != ps2.header.frame_id:
			ps2 = self.tf_listener.transformPose(target_frame=ps1.header.frame_id, ps=ps2)
		return math.hypot(ps1.pose.position.x-ps2.pose.position.x, ps1.pose.position.y-ps2.pose.position.y)
	
	def map_to_world(self, x, y):
		if not self.map:
			raise NotEnoughDataException('no map data received.')
		stamped = PoseStamped()
		stamped.header.frame_id = self.map.header.frame_id
		stamped.pose.position.x = x * self.map.info.resolution
		stamped.pose.position.y = y * self.map.info.resolution
		stamped.pose.orientation.w = 1
		origin_transform = pm.fromMsg(self.map.info.origin)
		center_transform = pm.fromMsg(stamped.pose)
		stamped.pose = pm.toMsg(origin_transform * center_transform)
		return stamped
	
	def estimate_center_from_map(self):
		if not self.map:
			raise NotEnoughDataException('no map data received.')
		map = self.map
		# estimate center cell based on free cells
		mean_x = 0
		mean_y = 0
		counter = 0
		for x in range(0, map.info.width, 2):
			for y in range(0, map.info.height, 2):
				index = y * map.info.width + x
				if map.data[index] == -1: #unknown
					continue
				if map.data[index] < 30: # free
					mean_x += x
					mean_y += y
					counter += 1
		stamped = self.map_to_world(mean_x / float(counter), mean_y / float(counter))
		msg = MarkerArray()
		msg.markers.append(self.create_pose_marker(stamped))
		msg.markers[-1].ns += '/map_center'
		self.visualization_publisher.publish(msg)
		return stamped
	
	def estimate_center_from_worldmodel(self):
		if not self.model:
			raise NotEnoughDataException('no worldmodel data received.')
		min_x = 1000
		max_x = -1000
		min_y = 1000
		max_y = -1000
		for object in self.model.objects:
			min_x = min(object.pose.pose.position.x, min_x)
			max_x = max(object.pose.pose.position.x, max_x)
			min_y = min(object.pose.pose.position.y, min_y)
			max_y = max(object.pose.pose.position.y, max_y)
		stamped = PoseStamped()
		stamped.header.frame_id = self.map.header.frame_id
		stamped.pose.position.x = (max_x + min_x) / 2.0
		stamped.pose.position.y = (max_y + min_y) / 2.0
		stamped.pose.orientation.w = 1
		msg = MarkerArray()
		msg.markers.append(self.create_pose_marker(stamped))
		msg.markers[-1].ns += '/model_center'
		msg.markers[-1].color.g = 0.8
		self.visualization_publisher.publish(msg)
		return stamped
	
	def sample_scan_pose(self):
		center = self.estimate_center_from_map()
		if not center:
			raise NotEnoughDataException('no map data recieved')
	
	def get_number_pose(self, number):
		number_class = 'number_banner_{}'.format(number)
		if number_class not in self.numbers:
			raise NumberNotFoundException(number)
		if len(self.numbers[number_class]) == 0:
			raise NumberNotFoundException(number)
		confidence_sorted = sorted(self.numbers[number_class].values(), lambda object: object.info.support)
		return confidence_sorted[0]
	
	
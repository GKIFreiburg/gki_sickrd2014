#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import random
import copy

import tf
import tf_conversions.posemath as pm
import PyKDL
from gki_sickrd_task.tools import Tools
from gki_sickrd_task.params import Params
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Empty
from hector_worldmodel_msgs.msg import ObjectModel, Object, ObjectInfo

class NotEnoughDataException(Exception):
	def __init__(self, message):
		self.message = message

class Percepts(object):
	def __init__(self):
		self.tools = Tools()
		self.model = None
		self.worldmodel_subscriber = rospy.Subscriber('/worldmodel/objects', ObjectModel, self.worldmodel_cb)
		self.estimated_map_center = None
		self.map = None
		self.map_center_need_update = False
		self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
		self.cube = None
		self.cube_subscriber = rospy.Subscriber('/cube_sensor', Bool, self.cube_sensor_cb)
		self.barcode = None
		self.barcode_subscriber = rospy.Subscriber('/barcode_processing/barcode', Int32, self.barcode_cb)
		self.barcode_enable_service = rospy.ServiceProxy('/barcode_processing/enable_detection', Empty)
		self.barcode_disable_service = rospy.ServiceProxy('/barcode_processing/disable_detection', Empty)
		for service in [self.barcode_enable_service, self.barcode_disable_service]:
			rospy.loginfo('waiting for service {}...'.format(service.resolved_name))
			service.wait_for_service()
			rospy.loginfo('service {} is now available.'.format(service.resolved_name))
		rospy.loginfo('percepts initialized.')

	def map_to_world(self, x, y):
		if not self.map:
			raise NotEnoughDataException('no map message received.')
		stamped = PoseStamped()
		stamped.header.frame_id = self.map.header.frame_id
		stamped.pose.position.x = x * self.map.info.resolution
		stamped.pose.position.y = y * self.map.info.resolution
		stamped.pose.orientation.w = 1
		origin_transform = pm.fromMsg(self.map.info.origin)
		center_transform = pm.fromMsg(stamped.pose)
		stamped.pose = pm.toMsg(origin_transform * center_transform)
		return stamped

	def cube_loaded(self):
		if not self.cube:
			raise NotEnoughDataException('no cube sensor message received.')
		return self.cube.value

	def current_number(self):
		if not self.barcode:
			raise NotEnoughDataException('no barcode detection message received.')
		if self.barcode.value == -1:
			raise NotEnoughDataException('no barcode detected.')
		return self.barcode.value
	
	def clear_number(self):
		self.barcode = None

	def estimate_center_from_map(self):
		if not self.map:
			raise NotEnoughDataException('no map message received.')
		if self.estimated_map_center and not self.map_center_need_update:
			return self.estimated_map_center
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
		self.estimated_map_center = self.map_to_world(mean_x / float(counter), mean_y / float(counter))
		msg = MarkerArray()
		msg.markers.append(self.tools.create_pose_marker(self.estimated_map_center, ns='worldmodel/map_center', z_offset=0.2))
		msg.markers[-1].color.b = 0.8
		msg.markers[-1].type = Marker.CYLINDER
		self.tools.visualization_publisher.publish(msg)
		self.map_center_need_update = False
		return self.estimated_map_center

	def estimate_center_from_worldmodel(self):
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
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
		msg.markers.append(self.tools.create_pose_marker(stamped, ns='worldmodel/model_center', z_offset=0.2))
		msg.markers[-1].color.b = 0.8
		msg.markers[-1].color.g = 0.8
		msg.markers[-1].type = Marker.CYLINDER
		self.tools.visualization_publisher.publish(msg)
		return stamped

	def visualize_worldmodel(self):
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
		msg = MarkerArray()
		# loading stations
		try:
			for station in self.get_loading_stations():
				stamped = PoseStamped()
				stamped.header.frame_id = self.map.header.frame_id
				stamped.pose = station.pose.pose
				msg.markers.append(self.tools.create_pose_marker(stamped, ns='loading_stations'))
				msg.markers[-1].color.r = 0.8
				msg.markers[-1].color.g = 0.8
				msg.markers[-1].color.b = 0.8
				msg.markers[-1].type = Marker.CUBE
				msg.markers[-1].id = len(msg.markers)
		except NotEnoughDataException:
			rospy.loginfo('no loading stations known.')
		# banners
		for number in range(10):
			try:
				banner = self.get_number_banner(number)
				stamped = PoseStamped()
				stamped.header.frame_id = self.map.header.frame_id
				stamped.pose = banner.pose.pose
				msg.markers.append(self.tools.create_pose_marker(stamped, ns='number_banners'))
				i = number + 4
				msg.markers[-1].color.r = i/9*0.4
				msg.markers[-1].color.g = i%9/3*0.4
				msg.markers[-1].color.b = i%3*0.4
				msg.markers[-1].type = Marker.CUBE
				msg.markers[-1].id = number
			except NotEnoughDataException:
				rospy.loginfo('number {} unknown.'.format(number))
		self.tools.visualization_publisher.publish(msg)

	def get_loading_stations(self):
		center = self.estimate_center_from_map()
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
		model = self.model
		lines = [object for object in self.model.objects if object.info.class_id == 'isolated_lines' and self.tools.xy_point_distance(object.pose.pose.position, center.pose.position) < Params.get().max_loading_station_distance_from_center]
		if len(lines) == 0:
			raise NotEnoughDataException('no known loading stations.')
		return lines

	def get_number_banner(self, number):
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
		number_class = 'number_banner_{}'.format(number)
		model = self.model
		banners = [object for object in self.model.objects if object.info.class_id == number_class]
		if len(banners) == 0:
			raise NotEnoughDataException('number not found: {}'.format(number))
		banners.sort(key=lambda object: object.info.support)
		return banners[0]

	def sample_approach_pose(self, stamped):
		center = self.estimate_center_from_map()
		if stamped.header.frame_id != center.header.frame_id:
			approach = self.tools.transform_pose(center.header.frame_id, stamped)
		else:
			approach = copy.deepcopy(stamped)
		map_yaw = math.atan2(approach.pose.position.y-center.pose.position.y, approach.pose.position.x-center.pose.position.x)
		if self.tools.xy_distance(center, approach) < Params.get().optimal_exploration_distance:
			# project outward
			approach.pose.position.x += Params.get().approach_distance * math.sin(map_yaw)
			approach.pose.position.y += Params.get().approach_distance * math.cos(map_yaw)
			rotation = tf.transformations.quaternion_from_euler(0, 0, map_yaw + math.pi)
		else:
			# project inward
			approach.pose.position.x -= Params.get().approach_distance * math.sin(map_yaw)
			approach.pose.position.y -= Params.get().approach_distance * math.cos(map_yaw)
			rotation = tf.transformations.quaternion_from_euler(0, 0, map_yaw + math.pi)
		approach.pose.orientation.x = rotation[0]
		approach.pose.orientation.y = rotation[1]
		approach.pose.orientation.z = rotation[2]
		approach.pose.orientation.w = rotation[3]
		return approach

	def sample_scan_pose(self):
		center = self.estimate_center_from_map()
		current = self.tools.get_current_pose()
		distance = 0.0
		scan_distance = Params.get().optimal_exploration_distance
		while distance < Params.get().min_travel_distance_for_rescan:
			map_yaw = self.tools.rnd.uniform(-math.pi, math.pi)
			center_distance = self.tools.rnd.uniform(scan_distance*0.75, scan_distance*1.33)
			scan = copy.deepcopy(center)
			scan.pose.position.x += center_distance * math.sin(map_yaw)
			scan.pose.position.y += center_distance * math.cos(map_yaw)
			distance = self.tools.xy_distance(current, scan)
		rotation = tf.transformations.quaternion_from_euler(0, 0, map_yaw + self.tools.rnd.uniform(-math.pi, math.pi)/2.0)
		scan.pose.orientation.x = rotation[0]
		scan.pose.orientation.y = rotation[1]
		scan.pose.orientation.z = rotation[2]
		scan.pose.orientation.w = rotation[3]
		return scan

	def enable_barcode_detection(self):
		self.barcode_enable_service.call()

	def disable_barcode_detection(self):
		self.barcode_disable_service.call()

	def worldmodel_cb(self, msg):
		self.model = msg

	def map_cb(self, msg):
		self.map = msg
		self.map_center_need_update = True

	def cube_sensor_cb(self, msg):
		self.cube = msg

	def barcode_cb(self, msg):
		self.barcode = msg


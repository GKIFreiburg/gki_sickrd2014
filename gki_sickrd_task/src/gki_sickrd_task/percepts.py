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
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Pose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Int32, Float32
from std_srvs.srv import Empty
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from hector_worldmodel_msgs.msg import ObjectModel, Object, ObjectInfo
from gki_sickrd_task.estop_guard import EstopGuard

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
		self.cube_number = -1
		self.detection_enabled = False
		self.barcode_subscriber = rospy.Subscriber('/barcode', Int32, self.barcode_cb)
		self.barcode_enable_service = rospy.ServiceProxy('/barcode_detection/enable_detection', Empty)
		self.barcode_disable_service = rospy.ServiceProxy('/barcode_detection/disable_detection', Empty)
		self.check_path_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
		for service in [self.barcode_enable_service, self.barcode_disable_service, self.check_path_service]:
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

	def check_path(self, start, goal):
		response = self.check_path_service.call(start=start, goal=goal)
		return len(response.plan.poses) > 0

	def cube_loaded(self):
		if not self.cube:
			raise NotEnoughDataException('no cube sensor message received.')
		return self.cube.data

	def current_number(self):
		if self.cube_number == -1:
			raise NotEnoughDataException('no barcode detected.')
		return self.cube_number

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

	def visualize_worldmodel(self, status_string=''):
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
		msg = MarkerArray()
		# loading stations
		id = 0
		try:
			for id, station in enumerate(self.get_loading_stations()):
				approach_pose = self.sample_approach_pose(station.pose.pose, station.header.frame_id)
				msg.markers.extend(self.tools.create_loading_station_markers(station, approach_pose, id))
		except NotEnoughDataException:
			rospy.loginfo('no loading stations known.')
		finally:
			for id in range(id+1, 10):
				msg.markers.extend(self.tools.create_loading_station_markers(id=id))
		# banners
		for number in range(10):
			try:
				banner = self.get_number_banner(number)
				approach = self.sample_approach_pose(banner.pose.pose, banner.header.frame_id)
				msg.markers.extend(self.tools.create_banner_markers(banner, approach, id=number))
			except NotEnoughDataException:
				#rospy.loginfo('number {} unknown.'.format(number))
				msg.markers.extend(self.tools.create_banner_markers(id=number))
		msg.markers.append(self.tools.create_status_marker(text=status_string))
		self.tools.visualization_publisher.publish(msg)

	def get_best_loading_station(self, current):
		all = self.get_loading_stations()
		good = []
		if Params().smart_loading_station_selection:
			for loading_station in all:
				approach_pose = self.sample_approach_pose(loading_station.pose.pose, loading_station.header.frame_id)
				valid = self.check_path(current, approach_pose)
				if valid:
					offset = Pose()
					offset.position.x = Params().approach_test_distance
					offset.orientation.w = 1
					goal = PoseStamped(header=current.header)
					goal.pose = self.tools.add_poses(approach_pose.pose, offset)
					valid = self.check_path(approach_pose, goal)
					if valid:
						good.append(loading_station)
		else:
			good = all
		good.sort(key=lambda object: self.tools.xy_point_distance(object.pose.pose.position, current.pose.position))
		return good[0]

	def get_loading_stations(self):
		center = self.estimate_center_from_map()
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
		model = self.model
		lines = [object for object in model.objects if object.info.class_id == 'isolated_lines' and self.tools.xy_point_distance(object.pose.pose.position, center.pose.position) < Params().max_loading_station_distance_from_center]
		if len(lines) == 0:
			raise NotEnoughDataException('no known loading stations.')
		return lines

	def get_closest_wall(self, point):
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
		model = self.model
		lines = [object for object in model.objects if object.info.class_id == 'connected_lines' and self.tools.xy_point_distance(object.pose.pose.position, point) < Params().approach_distance]
		if len(lines) == 0:
			raise NotEnoughDataException('no known walls.')
		lines.sort(key=lambda object: self.tools.xy_point_distance(object.pose.pose.position, point))
		return lines[0]
	
	def _fix_banner_orientation(self, banner):
		try:
			wall = self.get_closest_wall(banner.pose.pose.position)
			yaw = self.tools.get_yaw(wall.pose.pose)
			banner.pose.pose = self.tools.set_orientation_from_yaw(banner.pose.pose, yaw)
			return
		except NotEnoughDataException:
			pass
		center = self.estimate_center_from_map()
		angle = self.tools.get_facing_angle(center.pose, banner.pose.pose)
		if abs(angle) < math.pi/2.0:
			yaw = self.tools.get_yaw(banner.pose.pose)
			banner.pose.pose = self.tools.set_orientation_from_yaw(banner.pose.pose, yaw+math.pi)

	def get_number_banner(self, number):
		banners = self.get_number_banner_data(number)
		self._fix_banner_orientation(banners[0])
		return banners[0]

	def filter_conflicting_banners(self, number_objects):
		""" Adjust the support for all number objects that are at the same pose.

			If there is a dominating one, reduce the support for all others to be non-trustworthy.
			If there are multiple dominating ones, reduce all to avoid confusions."""
		# FIXME The second part might destroy some high support if the vision detects two
		# numbers consistently. Shouldn't happen?!?

		# For now we do this locally without clustering.
		# If we reduce the support of one, it can't dominate its neighbors any more,
		# so the order matters unless clustered beforehand.
		def get_neighbors(number_obj, objects):
			""" return all objects around number_obj (including that)
				that are in number_conflict_distance and
				also have at least min_trusted_support"""
			return [obj for obj in objects if
					self.tools.xy_point_distance(
						number_obj.pose.pose.position, obj.pose.pose.position) <= Params().number_conflict_distance
					and obj.info.support >= Params().min_trusted_support]
		for no in number_objects:
			if no.info.support < Params().min_trusted_support:  # this one is bad anyways
				continue
			neigh = get_neighbors(no, number_objects)
			if len(neigh) <= 1: # only we are in here
				continue
			# OK, we are trusted AND we have trusted neighbors
			# That is bad.
			neigh.sort(key = lambda obj: -obj.info.support)
			# If there is a single dominator, bring all others down
			if(neigh[0].info.support >= Params().domination_factor * neigh[1].info.support):
				for nn in neigh[1:]:
					rospy.loginfo("Reducing support below trusted for dominated by %s pose for %s, at %f %f. Had: %f support." % (neigh[0].info.class_id, nn.info.class_id, nn.pose.pose.position.x, nn.pose.pose.position.y, nn.info.support))
					nn.info.support = Params().min_trusted_support - 1
			else:
				# If the best one isn't a single dominator, bring all down - this position is confusing
				for nn in neigh:
					rospy.loginfo("Reducing support below trusted for conflicting poses for %s, at %f %f. Had: %f support." % (nn.info.class_id, nn.pose.pose.position.x, nn.pose.pose.position.y, nn.info.support))
					nn.info.support = Params().min_trusted_support - 1

	def get_number_banner_data(self, number):
		if not self.model:
			raise NotEnoughDataException('no worldmodel message received.')
		number_class = 'number_banner_{}'.format(number)
		model = self.model
		banners = [object for object in model.objects
				if object.info.class_id == number_class and object.info.support >= Params().min_valid_support]
		if len(banners) == 0:
			raise NotEnoughDataException('number not found: {}'.format(number))
		self.filter_conflicting_banners([object for object in model.objects
			if object.info.class_id.startswith("number_banner_")])
		banners.sort(key=lambda object: -object.info.support)
		return banners

	def sample_approach_pose(self, pose, frame_id):
		stamped = PoseStamped()
		stamped.header.frame_id = frame_id
		stamped.pose = pose
		return self.sample_approach_stamped(stamped)
		
	def sample_approach_stamped(self, stamped):
		center = self.estimate_center_from_map()
		if stamped.header.frame_id != center.header.frame_id:
			approach = self.tools.transform_pose(center.header.frame_id, stamped)
		else:
			approach = copy.deepcopy(stamped)
		if self.tools.xy_distance(center, stamped) < Params().max_loading_station_distance_from_center:
			yaw = self.tools.get_yaw(stamped.pose)
			projected = self.tools.set_orientation_from_yaw(stamped.pose, yaw)
			projected.position.z = 0
			offset = self.tools.set_orientation_from_yaw(Pose(), yaw)
			offset.position.x = Params().approach_distance
			approach.pose = self.tools.set_orientation_from_yaw(self.tools.add_poses(projected, offset), yaw+math.pi)
		else:
			yaw = self.tools.get_yaw(stamped.pose)
			try:
				yaw = self.get_direction_from_closest_wall(stamped.pose.position)
			except NotEnoughDataException:
				pass
			projected = self.tools.set_orientation_from_yaw(stamped.pose, yaw)
			projected.position.z = 0
			offset = self.tools.set_orientation_from_yaw(Pose(), yaw)
			offset.position.x = Params().approach_distance
			approach.pose = self.tools.set_orientation_from_yaw(self.tools.add_poses(projected, offset), yaw+math.pi)
		return approach
	
	def get_direction_from_closest_wall(self, point):
		closest_wall = self.get_closest_wall(point)
		return self.tools.get_yaw(closest_wall.pose.pose)
	
	def sample_scan_pose(self):
		rospy.loginfo('sampling scan pose')
		center = self.estimate_center_from_map()
		current = self.tools.get_current_pose()
		distance = 0.0
		scan_distance = Params().optimal_exploration_distance
		reachable = False
		sampled_poses = []
		for i in range(25):
			map_yaw = self.tools.rnd.uniform(-math.pi, math.pi)
			center_distance = self.tools.rnd.uniform(Params().min_travel_distance_for_rescan, scan_distance)
			scan = copy.deepcopy(current)
			scan.pose.position.x += center_distance * math.cos(map_yaw)
			scan.pose.position.y += center_distance * math.sin(map_yaw)
			rotation = tf.transformations.quaternion_from_euler(0, 0, map_yaw + self.tools.rnd.uniform(-math.pi, math.pi)/2.0)
			scan.pose.orientation.x = rotation[0]
			scan.pose.orientation.y = rotation[1]
			scan.pose.orientation.z = rotation[2]
			scan.pose.orientation.w = rotation[3]
			distance = self.tools.xy_distance(current, scan)
			if self.check_path(current, scan):
				return scan
			sampled_poses.append(scan)
		self.tools.visualize_poses(sampled_poses, ns='failed scan poses')
		raise NotEnoughDataException('could not sample exploration pose.')

	def enable_barcode_detection(self):
		if not self.detection_enabled:
			rospy.loginfo('enable barcode detection')
			self.detection_enabled = True
			self.barcode_enable_service.call()

	def disable_barcode_detection(self):
		if self.detection_enabled:
			rospy.loginfo('disable barcode detection')
			self.detection_enabled = False
			self.barcode_disable_service.call()

	def worldmodel_cb(self, msg):
		self.model = msg

	def map_cb(self, msg):
		self.map = msg
		self.map_center_need_update = True

	def cube_sensor_cb(self, msg):
		self.cube = msg
		if not self.cube.data and self.cube_number != -1:
			rospy.loginfo('clear cube number')
			self.cube_number = -1
		if self.cube.data and self.cube_number == -1:
			self.enable_barcode_detection()

	def barcode_cb(self, msg):
		self.barcode = msg
		if self.barcode.data != -1 and self.cube.data:
			self.cube_number = self.barcode.data
			rospy.loginfo('new cube number: {}'.format(self.cube_number))
			self.disable_barcode_detection()


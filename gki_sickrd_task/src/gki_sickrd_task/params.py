#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy

class Singleton(type):
	_instances = {}
	def __call__(cls, *args, **kwargs):
		if cls not in cls._instances:
			cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
		return cls._instances[cls]

class Params(object):
	__metaclass__ = Singleton

	def __init__(self):
		self.update()

	def update(self):
		self.__dict__ = rospy.get_param('~')

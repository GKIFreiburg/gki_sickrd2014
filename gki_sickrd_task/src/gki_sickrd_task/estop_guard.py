#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy

from std_msgs.msg import Bool

class EstopGuard(object):
	_instances = []
	_changed_cb = None

	@staticmethod
	def initialize(estop_topics, estop_changed_cb):
		EstopGuard._instances = []
		EstopGuard._changed_cb = estop_changed_cb
		for topic in estop_topics:
			EstopGuard._instances.append(EstopGuard(topic))
		rospy.loginfo('estop guard initialized.')

	@staticmethod
	def _any_triggered():
		for instance in EstopGuard._instances:
			if instance._triggered(): 
				return True
		return False

	def __init__(self, topic):
		self.estop = None
		rospy.loginfo('subscribing to estop topic {}'.format(topic))
		self.subscriber = rospy.Subscriber(topic, Bool, self._estop_cb)

	def _triggered(self):
		if not self.estop:
			return False
		return not self.estop.data
	
	def _estop_cb(self, msg):
		old = EstopGuard._any_triggered()
		self.estop = msg
		new = EstopGuard._any_triggered()
		if old != new:
			EstopGuard._changed_cb(new)

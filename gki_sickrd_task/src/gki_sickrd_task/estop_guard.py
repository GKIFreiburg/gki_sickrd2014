#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy

from std_msgs.msg import Bool
from gki_sickrd_task.params import Params

class EstopGuard(object):
	_instances = None
	_observers = None

	@staticmethod
	def add_callback(estop_changed_cb):
		if not EstopGuard._observers:
			EstopGuard._observers = []
			EstopGuard._instances = []
			for topic in Params().estop_topics:
				EstopGuard._instances.append(EstopGuard(topic))
			rospy.loginfo('estop guard initialized.')
		EstopGuard._observers.append(estop_changed_cb)

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
			for notify_cb in EstopGuard._observers:
				notify_cb(new)

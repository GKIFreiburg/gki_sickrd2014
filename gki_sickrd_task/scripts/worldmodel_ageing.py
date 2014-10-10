#!/usr/bin/env python

import roslib; roslib.load_manifest("gki_sickrd_task")
import rospy
import math
import threading

from gki_sickrd_task.estop_guard import EstopGuard
from gki_sickrd_task.params import Params

class WorldmodelAgeing(object):
	def __init__(self):
		self.worldmodel_ageing_publisher = rospy.Publisher('/worldmodel/object_ageing', Float32)
		self.stop = False
		self.last_ageing = rospy.Time.now()
		rospy.Timer(rospy.Duration(1.0), self.ageing_cb)
		EstopGuard.add_callback(self.estop_changed_cb)

	def estop_changed_cb(self, stop):
		self.stop = stop

	def ageing_cb(self, event):
		if not self.stop:
			self.worldmodel_ageing_publisher.publish(Float32(data=Params().worldmodel_ageing_rate))

if __name__ == "__main__":
	rospy.init_node("worldmodel_ageing")
	ageing = WorldmodelAgeing()
	rospy.spin()


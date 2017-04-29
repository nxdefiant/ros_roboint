#!/usr/bin/env python
import roslib; roslib.load_manifest('roboint')
import rospy
from math import *
from sensor_msgs.msg import Range
from roboint.msg import Inputs


class RoboExplorer:
	def __init__(self):
		rospy.init_node('robo_explorer')

		self.last_time = rospy.Time.now()
		self.pub_sonar = rospy.Publisher("sonar", Range, queue_size=16)
		rospy.Subscriber("ft/get_inputs", Inputs, self.inputsReceived)

		rospy.spin()
	
	def inputsReceived(self, msg):
		current_time = rospy.Time.now()

		#self.update_odometry(msg, current_time)
		if (current_time - self.last_time).to_nsec() > 100e6: # send every 100ms
			#self.send_odometry(msg, current_time)
			self.send_range(msg, current_time)
			self.last_time = current_time

	def send_range(self, msg, current_time):
		# ultra sonic range finder
		scan = Range()
		scan.header.stamp = current_time
		scan.header.frame_id = "forward_sensor"
		scan.radiation_type = 0
		scan.field_of_view = 60*pi/180
		scan.min_range = 0.0
		scan.max_range = 4.0
		scan.range = msg.d1/100.0
		self.pub_sonar.publish(scan)

if __name__ == '__main__':
	RoboExplorer()

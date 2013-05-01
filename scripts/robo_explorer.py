#!/usr/bin/env python
import roslib; roslib.load_manifest('roboint')
import rospy
import tf
from math import sin, cos
from geometry_msgs.msg import Twist, TransformStamped, Point32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from roboint.msg import Motor
from roboint.msg import Inputs


class RoboExplorer:
	def __init__(self):
		rospy.init_node('robo_explorer')
		
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelReceived)
		rospy.Subscriber("ft/get_inputs", Inputs, self.inputsReceived)

		self.pub_motor = rospy.Publisher("ft/set_motor", Motor)
		self.pub_cloud = rospy.Publisher("point_cloud", PointCloud)
		self.pub_odom = rospy.Publisher("odom", Odometry)

		self.speed = (0, 0)
		self.x = 0
		self.y = 0
		self.th = 0
		self.last_in = [0, 0]
		self.odom_broadcaster = tf.TransformBroadcaster()
		self.last_time = rospy.Time.now()

		rospy.spin()

	def inputsReceived(self, msg):
		current_time = rospy.Time.now()

		cloud = PointCloud()
		cloud.header.stamp = current_time
		cloud.header.frame_id = "sensor_frame"
		cloud.points.append(Point32(msg.d1/10.0, 0, 0))
		self.pub_cloud.publish(cloud)

		dt = (current_time - self.last_time).to_sec();
		in_now = msg.input[1:3]
		in_diff = [abs(a - b) for a, b in zip(in_now, self.last_in)] # get changed inputs
		if self.speed[0] < 0:
			in_diff[0] = -in_diff[0]
		if self.speed[1] < 0:
			in_diff[1] = -in_diff[1]

		self.diff_to_angle = 0.1 # TODO
		diff_si = in_diff # TODO

		delta_th = (diff_si[0] - diff_si[1]) * self.diff_to_angle
		self.th += delta_th

		movement = (diff_si[0] + diff_si[1])/2.0
		delta_x = cos(self.th)*movement
		delta_y = sin(self.th)*movement
		self.x += delta_x
		self.y += delta_y

		# speeds
		vx = delta_x / dt
		vy = delta_y / dt
		vth = delta_th / dt

		# first, we'll publish the transform over tf
		odom_trans = TransformStamped()
		odom_trans.header.stamp = current_time
		odom_trans.header.frame_id = "odom"
		odom_trans.child_frame_id = "base_link"
		odom_trans.transform.translation.x = self.x
		odom_trans.transform.translation.y = self.y
		odom_trans.transform.translation.z = 0.0
		odom_trans.transform.rotation = self.th

		## send the transform
		#self.odom_broadcaster.sendTransform(odom_trans);

		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"

		# set the position
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation = self.th

		# set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx
		odom.twist.twist.linear.y = vy
		odom.twist.twist.angular.z = vth

		# publish the message
		self.pub_odom.publish(odom)

		self.last_time = current_time
		self.last_in = in_now


	def cmdVelReceived(self, msg):
		trans = msg.linear.x
		rot = msg.angular.z

		# speed steps = 7
		# max trans = 0.1m/s
		# max rot = 0.29rad/s

		speed_l = int(trans*7/0.1 - rot*7/0.29)
		speed_r = int(trans*7/0.1 + rot*7/0.29)
		if speed_l < -7: speed_l = -7
		elif speed_l > 7: speed_l = 7
		if speed_r < -7: speed_r = -7
		elif speed_r > 7: speed_r = 7

		outmsg = Motor()
		outmsg.num = 1
		outmsg.speed = speed_l
		self.pub_motor.publish(outmsg)
		
		outmsg = Motor()
		outmsg.num = 2
		outmsg.speed = speed_r
		self.pub_motor.publish(outmsg)

		self.speed = (speed_l, speed_r)

if __name__ == '__main__':
	RoboExplorer()

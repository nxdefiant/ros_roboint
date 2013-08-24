#!/usr/bin/env python
import roslib; roslib.load_manifest('roboint')
import rospy
import tf
from math import sin, cos, pi
from geometry_msgs.msg import Twist, TransformStamped, Point32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from roboint.msg import Motor
from roboint.msg import Inputs


class RoboExplorer:
	def __init__(self):
		rospy.init_node('robo_explorer')
		
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelReceived)
		rospy.Subscriber("ft/get_inputs", Inputs, self.inputsReceived)

		self.pub_motor = rospy.Publisher("ft/set_motor", Motor)
		self.pub_scan = rospy.Publisher("scan", LaserScan)
		self.pub_odom = rospy.Publisher("odom", Odometry)

		self.wheel_dist = 0.188 # 18.8cm
		self.wheel_size = 0.052*0.5 # 5.1cm gear ration=0.5
		self.speed = (0, 0)
		self.x = 0
		self.y = 0
		self.alpha = 0
		self.last_in = [0, 0]
		self.tf_broadcaster = tf.TransformBroadcaster()
		self.last_time = rospy.Time.now()

		rospy.spin()

	def inputsReceived(self, msg):
		current_time = rospy.Time.now()

		self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), current_time, "map", "base_link");
		self.send_odometry(msg, current_time)
		self.send_laser_scan(msg, current_time)

		self.last_time = current_time

	def send_odometry(self, msg, current_time):
		dt = (current_time - self.last_time).to_sec();
		in_now = msg.input[1:3]
		in_diff = [abs(a - b) for a, b in zip(in_now, self.last_in)] # get changed inputs
		if self.speed[0] < 0:
			in_diff[0] = -in_diff[0]
		if self.speed[1] < 0:
			in_diff[1] = -in_diff[1]

		dist_dir = (in_diff[1] - in_diff[0])*self.wheel_size*pi/8 # steps_changed in different direction => m
		delta_alpha = dist_dir/self.wheel_dist

		dist = (in_diff[0] + in_diff[1])/2.0*self.wheel_size*pi/8 # steps_changed same direction => m

		delta_x = cos(self.alpha + delta_alpha/2)*dist
		delta_y = sin(self.alpha + delta_alpha/2)*dist

		self.alpha += delta_alpha
		if self.alpha > 2*pi:
			self.alpha -= 2*pi
		elif self.alpha < -2*pi:
			self.alpha += 2*pi
		self.x += delta_x
		self.y += delta_y
		self.last_in = in_now

		# speeds
		vx = delta_x / dt
		vy = delta_y / dt
		valpha = delta_alpha / dt

		# first, we'll publish the transform over tf
		self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), current_time, "odom", "base_link");

		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "/odom"

		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.alpha)

		# set the position
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = odom_quat[0]
		odom.pose.pose.orientation.y = odom_quat[1]
		odom.pose.pose.orientation.z = odom_quat[2]
		odom.pose.pose.orientation.w = odom_quat[3]

		# set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx
		odom.twist.twist.linear.y = vy
		odom.twist.twist.angular.z = valpha

		# publish the message
		self.pub_odom.publish(odom)
		
	def send_laser_scan(self, msg, current_time):
		# first, we'll publish the transform over tf
		self.tf_broadcaster.sendTransform((0.06, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), current_time, "base_link", "scan");

		# actually ultra sonic range finder
		scan = LaserScan()
		scan.header.stamp = current_time
		scan.header.frame_id = "/scan"
		scan.angle_min = -pi/4;
		scan.angle_max = pi/4;
		scan.angle_increment = pi/4;
		scan.time_increment = 0.01;
		scan.range_min = 0.0;
		scan.range_max = 4.0;
		for i in range(3):
			scan.ranges.append(msg.d1/100.0)
		scan.intensities.append(0.5)
		scan.intensities.append(1.0)
		scan.intensities.append(0.5)
		self.pub_scan.publish(scan)

	# test with rostopic pub -1 cmd_vel geometry_msgs/Twist '[0, 0, 0]' '[0, 0, 0]'
	def cmdVelReceived(self, msg):
		trans = msg.linear.x
		rot = msg.angular.z # rad/s

		# handle rotation as offset to speeds
		speed_offset = (rot * self.wheel_dist)/2.0 # m/s

		# handle translation
		speed_l = 0
		wish_speed_left = trans - speed_offset
		if abs(wish_speed_left) > 1.7/64.3:
			speed_l = 64.3*abs(wish_speed_left) - 1.7
			if wish_speed_left < 0:
				speed_l*=-1
		speed_r = 0
		wish_speed_right = trans + speed_offset
		if abs(wish_speed_right) > 1.7/64.3:
			speed_r = 64.3*abs(wish_speed_right) - 1.7
			if wish_speed_right < 0:
				speed_r*=-1

		# check limits
		if speed_l < -7: speed_l = -7
		elif speed_l > 7: speed_l = 7
		if speed_r < -7: speed_r = -7
		elif speed_r > 7: speed_r = 7

		outmsg = Motor()
		outmsg.num = 1
		outmsg.speed = round(speed_l)
		self.pub_motor.publish(outmsg)
		
		outmsg = Motor()
		outmsg.num = 2
		outmsg.speed = round(speed_r)
		self.pub_motor.publish(outmsg)

		self.speed = (speed_l, speed_r)

if __name__ == '__main__':
	RoboExplorer()

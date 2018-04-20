#!/usr/bin/env python

import rospy
import utm

import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf

"""gps_fix_data = sensor_msgs.msg.NavSatFix()
gps_vel_data = geometry_msgs.msg.TwistStamped()
imu_data = sensor_msgs.msg.Imu()
odom_data = nav_msgs.msg.Odometry()
old_imu_time = 0.0
pub = rospy.Publisher("/odom", nav_msgs.msg.Odometry, queue_size=10)
tf_broadcaster = tf2_ros.TransformBroadcaster()
odom_data.header.frame_id="odom"
odom_data.child_frame_id="base_link"
tf_msg = geometry_msgs.msg.TransformStamped()

def listener():
	rospy.Subscriber("/fix", sensor_msgs.msg.NavSatFix, recv, callback_args=0)
	rospy.Subscriber("/vel", geometry_msgs.msg.TwistStamped, recv, callback_args=1)
	rospy.Subscriber("/imu_data", sensor_msgs.msg.Imu, recv, callback_args=2)
	rospy.spin()

def recv(data, arg):
	global gps_fix_data, gps_vel_data, imu_data, pub, odom_data
	if arg==0:
		gps_fix_data = data
		impose()
	elif arg==1:
		gps_vel_data = data
		impose()
	elif arg==2:
		old_imu_time = imu_data.header.stamp.to_sec()
		imu_data = data
		integrate()

	pub.publish(odom_data)
	update_transform()
	#rate.sleep()


def impose():
	global gps_vel_data, gps_fix_data, odom_data
	if gps_fix_data.header.stamp==gps_vel_data.header.stamp or gps_fix_data.status.status== -1:
		return
	lat = gps_fix_data.latitude
	lon = gps_fix_data.longitude
	alt = gps_fix_data.altitude
	x,y = utm.from_latlon(lat, lon)[:2]
	odom_data.pose.pose.position.x = x-393915.5005
	odom_data.pose.pose.position.y = y-1381185.75117
	#odom_data.pose.pose.position.z = alt
	odom_data.pose.covariance[0] = gps_fix_data.position_covariance[0]
	odom_data.pose.covariance[7] = gps_fix_data.position_covariance[4]
	odom_data.pose.covariance[14] = gps_fix_data.position_covariance[8]
	odom_data.twist.twist.linear.x = gps_vel_data.twist.linear.x
	odom_data.twist.twist.linear.y = gps_vel_data.twist.linear.y
	#odom_data.twist.twist.linear.z = gps_vel_data.twist.linear.z
	odom_data.twist.covariance[0] = 0
	odom_data.twist.covariance[7] = 0
	odom_data.twist.covariance[14] = 0

def integrate():
	time = imu_data.header.stamp.to_sec()-old_imu_time
	odom_data.pose.pose.orientation = imu_data.orientation
	odom_data.pose.covariance[21] = imu_data.orientation_covariance[0]
	odom_data.pose.covariance[28] = imu_data.orientation_covariance[4]
	odom_data.pose.covariance[35] = imu_data.orientation_covariance[8]
	odom_data.twist.twist.angular.x = imu_data.angular_velocity.x
	odom_data.twist.twist.angular.y = imu_data.angular_velocity.y
	#odom_data.twist.twist.angular.z = imu_data.angular_velocity.z
	odom_data.twist.covariance[21] = imu_data.angular_velocity_covariance[0]
	odom_data.twist.covariance[28] = imu_data.angular_velocity_covariance[4]
	odom_data.twist.covariance[35] = imu_data.angular_velocity_covariance[8]
	odom_data.twist.twist.linear.x += imu_data.linear_acceleration.x*time
	odom_data.twist.twist.linear.y += imu_data.linear_acceleration.y*time
	#odom_data.twist.twist.linear.z += imu_data.linear_acceleration.z*time
	odom_data.twist.covariance[0] += imu_data.linear_acceleration_covariance[0]
	odom_data.twist.covariance[7] += imu_data.linear_acceleration_covariance[4]
	odom_data.twist.covariance[14] += imu_data.linear_acceleration_covariance[8]
	odom_data.pose.pose.position.x += odom_data.twist.twist.linear.x
	odom_data.pose.pose.position.y += odom_data.twist.twist.linear.y
	#odom_data.pose.pose.position.z += odom_data.twist.twist.linear.z
	odom_data.pose.covariance[0] += odom_data.twist.covariance[0]
	odom_data.pose.covariance[7] += odom_data.twist.covariance[7]
	odom_data.pose.covariance[14] += odom_data.twist.covariance[14]
	rate.sleep()

def update_transform():
	global tf_msg
	tf_msg.header = odom_data.header
	tf_msg.child_frame_id = "base_link"
	tf_msg.transform.translation.x = odom_data.pose.pose.position.x
	tf_msg.transform.translation.y = odom_data.pose.pose.position.y
	tf_msg.transform.translation.z = odom_data.pose.pose.position.z
	tf_msg.transform.rotation = odom_data.pose.pose.orientation
	tf_broadcaster.sendTransform(tf_msg)

rospy.init_node("filter")
rate = rospy.Rate(100)

listener()"""

filter_gps_accept = [[True, True, False],
					[False, False, False],
					[True, True, False],
					[False, False, False],
					[False, False, False]]

filter_imu_accept = [[False, False, False],
					[True, True, True],
					[False, False, False],
					[True, True, True],
					[True, True, False]]

#TODO Apply data according to the Filter Accepting Matrices


class ROSFilter(object):

	def __init__(self, odom_topic_name, rate):
		self.imu_data = sensor_msgs.msg.Imu()
		self.fix_data = sensor_msgs.msg.NavSatFix()
		self.vel_data = geometry_msgs.msg.TwistStamped()
		self.odom_data = nav_msgs.msg.Odometry()
		#self.imu_accept = filter_imu_accept
		#self.gps_accept = filter_gps_accept
		rospy.init_node("localization_filter")
		self.odom_publisher = rospy.Publisher(odom_topic_name, nav_msgs.msg.Odometry, queue_size=10)
		self.rate = None
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()
		if rate!=0:
			self.rate = rospy.Rate(rate)

	def impose_gps(self):
		#if self.fix_data.header.stamp!=self.vel_data.header.stamp or self.fix_data.status.status== -1:
		#	return
		self.odom_data.header.frame_id = self.fix_data.header.frame_id
		gps_pose = utm.from_latlon(self.fix_data.latitude, self.fix_data.longitude)[:2]
		self.odom_data.pose.pose.position.x = gps_pose[0]
		self.odom_data.pose.pose.position.y = gps_pose[1]
		#self.odom_data.pose.pose.position.z = self.fix_data.altitude
		self.odom_data.pose.covariance[0] = self.fix_data.position_covariance[0]
		self.odom_data.pose.covariance[7] = self.fix_data.position_covariance[4]
		self.odom_data.pose.covariance[14] = self.fix_data.position_covariance[8]
		self.odom_data.twist.twist.linear.x = self.vel_data.twist.linear.x
		self.odom_data.twist.twist.linear.y = self.vel_data.twist.linear.y
		#self.odom_data.twist.twist.linear.z = self.vel_data.twist.linear.z
		self.odom_data.twist.covariance[0] = 0
		self.odom_data.twist.covariance[7] = 0
		self.odom_data.twist.covariance[14] = 0


	def listen(self, fix_topic_name, vel_topic_name, imu_topic_name):
		rospy.Subscriber(fix_topic_name, sensor_msgs.msg.NavSatFix, self.callback, callback_args=0)
		rospy.Subscriber(vel_topic_name, geometry_msgs.msg.TwistStamped, self.callback, callback_args=1)
		rospy.Subscriber(imu_topic_name, sensor_msgs.msg.Imu, self.callback, callback_args=2)
		rospy.spin()

	def callback(self, data, arg):
		if arg==0:
			self.get_fix_data(data)
			self.impose_gps()

		elif arg==1:
			self.get_vel_data(data)
			self.impose_gps()
		elif arg==2:
			self.get_imu_data(data)
		rospy.loginfo("Hello")
		self.tf_publish()
		self.publish()

	def tf_publish(self):
		tf_msg = geometry_msgs.msg.TransformStamped()
		tf_msg.header = self.odom_data.header
		tf_msg.child_frame_id = "base_link"
		tf_msg.transform.translation.x = self.odom_data.pose.pose.position.x
		tf_msg.transform.translation.y = self.odom_data.pose.pose.position.y
		tf_msg.transform.translation.z = self.odom_data.pose.pose.position.z
		tf_msg.transform.rotation = self.odom_data.pose.pose.orientation
		self.tf_broadcaster.sendTransform(tf_msg)

	def publish(self):
		while not rospy.is_shutdown():
			self.odom_publisher.publish(self.odom_data)
			if self.rate!=None:
				self.rate.sleep()


	def get_fix_data(self, fix_data):
		self.fix_data = fix_data

	def get_vel_data(self, vel_data):
		self.vel_data = vel_data
		self.impose_gps()

	def integrate_imu(self, old_imu_time):
		time = self.imu_data.header.stamp.to_sec()-old_imu_time
		rospy.loginfo(time)
		self.odom_data.pose.pose.orientation = self.imu_data.orientation
		self.odom_data.pose.covariance[21] = self.imu_data.orientation_covariance[0]
		self.odom_data.pose.covariance[28] = self.imu_data.orientation_covariance[4]
		self.odom_data.pose.covariance[35] = self.imu_data.orientation_covariance[8]
		self.odom_data.twist.twist.angular.x = self.imu_data.angular_velocity.x
		self.odom_data.twist.twist.angular.y = self.imu_data.angular_velocity.y
		#self.odom_data.twist.twist.angular.z = self.imu_data.angular_velocity.z
		self.odom_data.twist.covariance[21] = self.imu_data.angular_velocity_covariance[0]
		self.odom_data.twist.covariance[28] = self.imu_data.angular_velocity_covariance[4]
		self.odom_data.twist.covariance[35] = self.imu_data.angular_velocity_covariance[8]
		self.odom_data.twist.twist.linear.x += self.imu_data.linear_acceleration.x*time
		self.odom_data.twist.twist.linear.y += self.imu_data.linear_acceleration.y*time
		#self.odom_data.twist.twist.linear.z += self.imu_data.linear_acceleration.z*time
		self.odom_data.twist.covariance[0] += self.imu_data.linear_acceleration_covariance[0]
		self.odom_data.twist.covariance[7] += self.imu_data.linear_acceleration_covariance[4]
		self.odom_data.twist.covariance[14] += self.imu_data.linear_acceleration_covariance[8]
		self.odom_data.pose.pose.position.x += self.odom_data.twist.twist.linear.x
		self.odom_data.pose.pose.position.y += self.odom_data.twist.twist.linear.y
		#self.odom_data.pose.pose.position.z += odom_data.twist.twist.linear.z
		self.odom_data.pose.covariance[0] += self.odom_data.twist.covariance[0]
		self.odom_data.pose.covariance[7] += self.odom_data.twist.covariance[7]
		self.odom_data.pose.covariance[14] += self.odom_data.twist.covariance[14]

	def get_imu_data(self, imu_data):
		old_imu_time = self.imu_data.header.stamp.to_sec()
		self.imu_data = imu_data
		self.integrate_imu(old_imu_time)

	def get_current_state(self):
		return self.odom_data

rosfilter = ROSFilter("/odom", 100)
rosfilter.listen("/fix", "/vel", "/imu_data")
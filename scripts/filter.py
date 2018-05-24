#!/usr/bin/env python

import rospy
import utm

import sensor_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import tf2_ros
import numpy as np

#imu_data = sensor_msgs.msg.Imu()
count = 0
fix_data = sensor_msgs.msg.NavSatFix()
vel_data = geometry_msgs.msg.TwistStamped()
odom_data = nav_msgs.msg.Odometry()
odom_data.pose.pose.orientation.w = 1
odom_data.header.frame_id = "odom"
odom_data.child_frame_id = "base_link"
tf_br = tf2_ros.TransformBroadcaster()
#rate = rospy.Rate(20)

rospy.init_node("filter")
pub = rospy.Publisher("/odom", nav_msgs.msg.Odometry, queue_size=10)

def listener():
	rospy.Subscriber("/imu_data", sensor_msgs.msg.Imu, callback_imu)
	rospy.Subscriber("/fix", sensor_msgs.msg.NavSatFix, callback_fix)
	#Make the data type Vector3Stamped for gazebo and TwistStamped for real testing
	rospy.Subscriber("/vel", geometry_msgs.msg.Vector3Stamped, callback_vel)
	rospy.spin()

def callback_imu(data):
	global odom_data, count
	if (count!=0):
		time = (data.header.stamp-odom_data.header.stamp).to_sec()
		odom_data.twist.twist.linear.x += data.linear_acceleration.x*time
		odom_data.twist.twist.linear.y += data.linear_acceleration.y*time
		#odom_data.twist.twist.linear.z += data.linear_acceleration.z*time
		odom_data.twist.covariance[0] = data.linear_acceleration_covariance[0]
		odom_data.twist.covariance[7] = data.linear_acceleration_covariance[4]
		#odom_data.twist.covariance[14] = data.linear_acceleration_covariance[8]

		odom_data.pose.pose.position.x += odom_data.twist.twist.linear.x*time
		odom_data.pose.pose.position.y += odom_data.twist.twist.linear.y*time
		#odom_data.pose.pose.position.z += odom_data.twist.twist.linear.z*time
		odom_data.pose.covariance[0] += odom_data.twist.covariance[0]
		odom_data.pose.covariance[7] += odom_data.twist.covariance[7]
		odom_data.pose.covariance[14] += odom_data.twist.covariance[14]

		quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

		quat = np.linalg.norm(quat)

		odom_data.pose.pose.orientation = data.orientation

		odom_data.twist.twist.angular = data.angular_velocity
		odom_data.twist.covariance[21] = data.angular_velocity_covariance[0]
		odom_data.twist.covariance[28] = data.angular_velocity_covariance[4]
		odom_data.twist.covariance[35] = data.angular_velocity_covariance[8]
	count += 1
	odom_data.header.stamp = data.header.stamp
	publish_transform()


def callback_fix(data):
	global fix_data, vel_data
	fix_data = data
	if(fix_data.status.status>=0):
		utm_co = utm.from_latlon(fix_data.latitude, fix_data.longitude)
		#odom_data.pose.pose.position.x = utm_co[0]
		#odom_data.pose.pose.position.y = utm_co[1]
		#odom_data.pose.pose.position.z = fix_data.altitude
		odom_data.pose.covariance[0] = fix_data.position_covariance[0]
		odom_data.pose.covariance[7] = fix_data.position_covariance[4]
		odom_data.pose.covariance[14] = fix_data.position_covariance[8]
		odom_data.header.stamp = data.header.stamp
		publish_transform()

def callback_vel(data):
	global odom_data
	odom_data.twist.twist.linear.x = data.vector.x
	odom_data.twist.twist.linear.y = data.vector.y
	#odom_data.twist.twist.linear.z = data.twist.linear.z
	odom_data.header.stamp = data.header.stamp
	publish_transform()

def publish_transform():
	global odom_data, pub, tf_br
	pub.publish(odom_data)
	tf_msg = geometry_msgs.msg.TransformStamped()
	tf_msg.header.frame_id = "odom"
	tf_msg.child_frame_id = "base_link"
	tf_msg.header.stamp = odom_data.header.stamp
	tf_msg.transform.rotation = odom_data.pose.pose.orientation
	tf_msg.transform.translation.x = odom_data.pose.pose.position.x
	tf_msg.transform.translation.y = odom_data.pose.pose.position.y
	tf_br.sendTransform(tf_msg)

listener()
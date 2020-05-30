#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion

import numpy as np 
from matplotlib import pyplot as plt

class defineTarget():
	def __init__(self):
		rospy.init_node('generate_odom')
		self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
		self.child_frame_id = rospy.get_param('~child_frame_id','base_link')
		self.frame_id = rospy.get_param('~frame_id','world')
		rospy.Subscriber('/cmd_vel', Twist, self.cbCmdVel)
		self.time_start = rospy.Time.now()
		self.time_to_run = rospy.get_param('~time_to_run',5000)
		self.v_target = 0
		self.w_target = 0
		self.x = 1
		self.y = 1
		self.th = 0
		self.dt = 0.2
		fig = plt.figure()

	def update(self):
		time_duration = (rospy.Time.now() - self.time_start).to_sec()
		if time_duration > self.time_to_run:
			rospy.signal_shutdown('TimeOut Occurred for target definition')
		# print "velocities: {}, {}".format(self.v_target, self.w_target)
		self.x = self.x + self.v_target * np.cos(np.radians(self.th)) * self.dt
		self.y = self.y + self.v_target * np.sin(np.radians(self.th)) * self.dt
		self.th = self.th + self.w_target * self.dt
		odom_msg = self.pub_odometry(self.x, self.y, self.th)
		self.odom_pub.publish(odom_msg)
		# print "position: {}, {}, {}".format(self.x, self.y, self.th)
		# plt.plot(self.x, self.y, 'k*')
		# plt.xlim(0, 10)
		# plt.ylim(0, 8)
		# plt.gca().set_aspect('equal', adjustable='box')
		# plt.pause(0.0001)

	def pub_odometry(self, data_x, data_y, th):
		odom_msg = Odometry()
		odom_msg.header.stamp = rospy.Time.now()
		odom_msg.header.frame_id = self.frame_id
		odom_msg.child_frame_id = self.child_frame_id#child_frame_id
		odom_msg.pose.pose.position = Point(data_x, data_y, 0)
		odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,th))
		return odom_msg

	def spin(self):
		rospy.loginfo("Target Defined")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def cbCmdVel(self, msg):
		self.v_target = msg.linear.x
		self.w_target = msg.angular.z

	def shutdown(self):
		rospy.loginfo("Shutting Down defined Target")
		rospy.sleep(1)

def main():
	define_target = defineTarget();
	define_target.spin()

if __name__ == '__main__':
	main()


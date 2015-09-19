#!/usr/bin/env python

""" Have bot move parallel to wall at user-determined distance. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from sensor_msgs.msg import LaserScan
import math

class Controller:
	def __init__(self):
		rospy.init_node('wall_stop')
		rospy.Subscriber('/scan', LaserScan, self.charge, queue_size=1)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		self.command = Twist()
		self.threshold = 0.05
		# self.target_distance = rospy.get_param('~target_distance') # user-input distance from wall
		self.stop()

	def charge(self, scan):
		''' if parallel to wall, go forward.  if not, rotate until parallel '''

		if scan.ranges[105]==0 or scan.ranges[75]==0:
			return

		if (abs(scan.ranges[105] - scan.ranges[75]) < self.threshold):
			self.forward(0.05)
		elif (scan.ranges[105] - scan.ranges[75]) > 0:
			self.spin_right()
		else:
			self.spin_left()

	### Manipulating bot
	def forward(self, x):
		''' drive bot forward '''
		self.command.linear.x = x
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = 0	

	def spin_left(self):
		''' spin bot left '''
		self.command.linear.x = 0
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = 0.3			

	def spin_right(self):
		''' spin bot right '''
		self.command.linear.x = 0
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = -0.3	

	def stop(self):
		''' stop all bot motion '''
		self.command.linear.x = 0
		self.pub.publish(self.command)

	def drive(self):
		self.pub.publish(self.command)

controller = Controller()

while not rospy.is_shutdown():
	controller.drive()

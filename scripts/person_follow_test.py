#!/usr/bin/env python

""" Have bot follow a moving person. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from sensor_msgs.msg import LaserScan, Image
import math

class Controller:
	def __init__(self):
		rospy.init_node('person_stop')
		rospy.Subscriber('/scan', LaserScan, self.read_scan, queue_size=1)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		self.command = Twist()
		self.threshold = 1 # let's say you have to be within 1m to register
		self.stop()

	### Scenarios

	def read_scan(self,scan):
		''' read the scan from 310-365, 0-45 '''
		measure_left = scan.ranges[0:45]
		measure_right = scan.ranges[310:365]

		# variable = [[i, measure[i]] for i in range(len(measure)) if measure[i] > 0]

		detect_left = [i for i in range(len(measure_left)) if (measure_left[i] > 0) & (measure_left[i] < 2)]

		detect_right = [i for i in range(len(measure_right)) if (measure_right[i] > 0) & (measure_right[i] < 2)]

		detect = detect_left + detect_right

		if len(detect) != 0:
			self.forward(0.05)
		else:
			self.stop()

		# avg = sum(detect)/len(detect)


		# rotate until looking straight the obstacle

		# if len(detect) == 0:
		# 	pass
		# 	# self.forward(0.05)
		# elif avg > 0 & avg < 45:
		# 	self.left()
		# else:
		# 	self.right()

	def charge(self,scan):
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
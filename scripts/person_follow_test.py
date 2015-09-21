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
		measure_left = scan.ranges[45:10:-1]
		measure_right = scan.ranges[350:315:-1]


		# for j,item in enumerate(measure_left):
		# 	print type(measure_left)
		# 	if item > 2:
		# 		measure_left[j] = 0

		# for k,item in enumerate(measure_right):
		# 	print type(k)
		# 	if item > 2:
		# 		measure_right[k] = 0

		# filter_left = [0 for i in range(len(measure_left)) if measure_left[i] > 2]
		# filter_right = [0 for i in range(len(measure_right)) if measure_right[i] > 2]


		detect_left = sum([i for i in measure_left if i>0 and i<1.5])
		detect_right = sum([i for i in measure_right if i>0 and i<1.5])		
		# detect_right = sum(i>0 for i in measure_right)

		# print 'LEFT: ',detect_left
		# print 'RIGHT: ',detect_right
		
		if (abs(detect_left - detect_right)) < 5:
			print 'go fwd'
			self.forward(0.1)
		elif detect_left > detect_right:
			print 'spin left'
			self.spin_left(0.3)
		elif detect_right > detect_left:
			print 'spin right'
			self.spin_right(0.3)
		else:
			print 'stop'
			self.stop()

	def charge(self,scan):
		''' if parallel to wall, go forward.  if not, rotate until parallel '''

		if scan.ranges[105]==0 or scan.ranges[75]==0:
			return

		if (abs(scan.ranges[105] - scan.ranges[75]) < self.threshold): 
			# if the two values are close enough to each other, move forward
			self.forward(0.05)
		elif (scan.ranges[105] - scan.ranges[75]) > 0:
			# if ranges[105] is greater, spin right
			self.spin_right()
		else:
			# if ranges[75] is greater, spin left
			self.spin_left()

	### Manipulating bot

	def forward(self,speed):
		''' drive bot forward '''
		self.command.linear.x = speed
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = 0	

	def spin_left(self,speed):
		''' spin bot left '''
		self.command.linear.x = 0
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = speed			

	def spin_right(self,speed):
		''' spin bot right '''
		self.command.linear.x = 0
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = -speed	

	def stop(self):
		''' stop all bot motion '''
		self.command.linear.x = 0
		self.pub.publish(self.command)

	def drive(self):
		self.pub.publish(self.command)

controller = Controller()

while not rospy.is_shutdown():
	controller.drive()
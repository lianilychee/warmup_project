#!/usr/bin/env python

""" Have bot follow a walking person like a pet. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from sensor_msgs.msg import LaserScan
import math
from sklearn.cluster import AffinityPropagation
import numpy as np

speed_factor = .6
personal_space = .5

class Controller:
	def __init__(self):
		rospy.init_node('person_follow')
		rospy.Subscriber('/scan', LaserScan,
			self.evaluate_tracking_box, queue_size=1)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.command = Twist()
		self.target = [0, 1]
		# self.threshold = 0.05
		# self.target_distance = rospy.get_param('~target_distance') # user-input distance from wall
		self.stop()
		self.drive()

	def evaluate_tracking_box(self, scan):
		tr_points_left = [[i, scan.ranges[i]] for i in range(0, 90)]
		tr_points_right = [[i, scan.ranges[i]] for i in range(270, 360)]
		tr_points = tr_points_left + tr_points_right
		xy_points = [tr_to_xy(tr_points[i]) for i in range(len(tr_points))]
		limited_xy_points = self.define_tracking_box(xy_points)
		point_array = np.array(limited_xy_points)
		if len(point_array) > 2:
			af = AffinityPropagation(max_iter=20).fit(point_array)
			clusters = af.cluster_centers_
			print(clusters)
			xy_target = self.calculate_target(clusters)
			self.target = xy_to_tr(xy_target)
			print(self.target)
		else:
			print("None")

	def define_tracking_box(self, xy_points):
		return [p for p in xy_points if p[0] > 0 and p[0] < 2 and abs(p[1]) < .5]

	def calculate_target(self, clusters):
		two_closest = find_two_closest(clusters)
		return average_two_points(two_closest[0], two_closest[1])

	def update_command(self):
		if abs(self.target[0]) > 10 and abs(self.target[0]) < 90:
			if self.target[0] > 0:
				self.command.angular.z = .5
			if self.target[0] < 0:
				self.command.angular.z = -.5
			self.command.linear.x = 0
		elif abs(self.target[1]) - personal_space > .1:
			self.command.angular.z = 0
			self.command.linear.x = speed_factor * (self.target[1] - personal_space)

	def stop(self):
		''' stop all bot motion '''
		self.command.linear.x = 0
		self.pub.publish(self.command)

	def drive(self):
		self.pub.publish(self.command)

def tr_to_xy(pair):
	''' convert a theta, radius pair to an x, y pair '''
	angle, radius = pair[0], pair[1]
	x = radius * math.cos(math.radians(angle))
	y = radius * math.sin(math.radians(angle))
	return [x,y]

def xy_to_tr(pair):
	''' convert an x, y pair to a theta, radius pair '''
	x, y = pair[0], pair[1]
	theta = math.degrees(math.atan((y / x)))
	radius = math.sqrt(x ** 2 + y ** 2)
	return [theta, radius]

def distance(pt1, pt2):
    pt1 = np.array((pt1[0], pt1[1]))
    pt2 = np.array((pt2[0], pt2[1]))
    return np.linalg.norm(pt1-pt2)

def average_two_points(pt1, pt2):
	return [((pt1[0] + pt2[0]) / 2), ((pt1[1] + pt2[1]) / 2)]

def find_two_closest(point_list):
	closest_indices = []
	smallest_distance = 100
	for i in range(len(point_list)):
		for j in range(len(point_list)):
			dist = distance(point_list[i], point_list[j])
			if dist < smallest_distance:
				closest_indices = [i, j]
				smallest_distance = dist
	return [point_list[closest_indices[0]],point_list[closest_indices[1]]]






controller = Controller()

while not rospy.is_shutdown():
	controller.update_command()
	controller.drive()

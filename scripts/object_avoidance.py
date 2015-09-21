#!/usr/bin/env python

""" Have bot follow a walking person like a pet. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import heapq

speed_factor = .6
personal_space = .5

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

class Graph:
  def __init__(self):
    self.edges = {}
    self.weights = {}

  def neighbors(self, id):
    return self.edges[id]

  def cost(self, a, b):
      return self.weights.get(b, 0)

  def heuristic(self, a, b):
      (x1, y1) = a
      (x2, y2) = b
      return abs(x1 - x2) + abs(y1 - y2)

  def a_star_search(self, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
      current = frontier.get()

      if current == goal:
        break

      for next in self.neighbors(current):
        new_cost = cost_so_far[current] + self.cost(current, next)
        if next not in cost_so_far or new_cost < cost_so_far[next]:
          cost_so_far[next] = new_cost
          priority = new_cost + self.heuristic(goal, next)
          frontier.put(next, priority)
          came_from[next] = current

    return came_from, cost_so_far

class GraphGenerator:
  def __init__(self, start, goal):
    self.start = start
    self.goal = goal
    self.grid_spacing = 0.1
    self.padding = 0.2

  def calculate_bounds(self):
    (x1, y1) = self.start
    (x2, y2) = self.goal
    left_bound = (x1 if x1 < x2 else x2) - self.padding
    right_bound = (x1 if x1 > x2 else x2) + self.padding
    bottom_bound = (y1 if y1 < y2 else y2) - self.padding
    top_bound = (y1 if y1 > y2 else y2) + self.padding
    return [left_bound, right_bound, bottom_bound, top_bound]

  def generate(self):
    graph = Graph()
    [left_bound, right_bound, bottom_bound, top_bound] = self.calculate_bounds()
    hor_grids = abs(left_bound - right_bound) / self.grid_spacing
    vert_grids = abs(bottom_bound - top_bound) / self.grid_spacing
    for i in np.linspace(left_bound, right_bound, num=hor_grids, endpoint=True):
      for j in np.linspace(bottom_bound, top_bound, num=vert_grids, endpoint=True):


class Controller:
  def __init__(self):
    rospy.init_node('person_follow')
    rospy.Subscriber('/odom', Odometry, self.react_odom, queue_size=1)
    rospy.Subscriber('/scan', LaserScan,
      self.evaluate_tracking_box, queue_size=1)
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.command = Twist()
    self.target = [0, 1]
    # self.threshold = 0.05
    # self.target_distance = rospy.get_param('~target_distance') # user-input distance from wall
    self.stop()
    self.drive()

  def react_odom(self, odom):
    ''' callback function: react to odom info '''
    odom.pose.pose.position.x

  def react_scan(self, scan):
    scan.ranges[]

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


controller = Controller()

while not rospy.is_shutdown():
  controller.update_command()
  controller.drive()
#!/usr/bin/env python

""" Bot traverses from Point A to Point B, avoiding obstacles along the way. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import heapq


speed_factor = .6
personal_space = .5

class PriorityQueue:
  ''' the closest point is arranged first '''
  def __init__(self):
      self.elements = []
  
  def empty(self):
      return len(self.elements) == 0
  
  def put(self, item, priority):
      heapq.heappush(self.elements, (priority, item))
  
  def get(self):
      return heapq.heappop(self.elements)[1]


class Graph:
  ''' creates path for bot to follow '''  
  def __init__(self):
    self.edges = {}

  def add_edge(self, pt1, pt2):
    if pt1 not in self.edges:
      self.edges[pt1] = [pt2]
    else:
      self.edges[pt1].append(pt2)

  def remove_point(self, pt1):
    if pt1 in self.edges:
      for pt2 in self.edges[pt1]:
        self.edges[pt2].remove(pt1)
      self.edges.pop(pt1)

  def neighbors(self, pt):
    return self.edges[pt]

  def cost(self, pt1, pt2):
    (x1, y1) = pt1
    (x2, y2) = pt2
    return clean(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))

  def heuristic(self, a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

  def navigate(self, start, goal):
    came_from, cost_so_far = self.a_star_search(start, goal)
    return self.reconstruct_path(came_from, start, goal)

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

  def reconstruct_path(self, came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


class GraphGenerator:
  ''' generate the network of nodes, called by Controller '''
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
    for i in np.arange(left_bound, right_bound, self.grid_spacing):
      i = clean(i)
      for j in np.arange(bottom_bound, top_bound, self.grid_spacing):
        j = clean(j)
        if i != left_bound:
          graph.add_edge((i, j), (clean(i - self.grid_spacing), j))
        if i != right_bound:
          graph.add_edge((i, j), (clean(i + self.grid_spacing), j))
        if j != bottom_bound:
          graph.add_edge((i, j), (i, clean(j - self.grid_spacing)))
        if j != top_bound:
          graph.add_edge((i, j), (i, clean(j + self.grid_spacing)))
    return graph


class Controller:
  def __init__(self):
    rospy.init_node('person_follow')
    rospy.Subscriber('/odom', Odometry, self.hit_waypoint, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, self.evaluate_tracking_box, queue_size=1) # UNCOMMENT
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.command = Twist()
    self.target = [0, 1]
    self.threshold = 0.1
    self.stop()
    self.drive()

  def recalculate(self,odom,waypoint):
    ''' define waypoint in relation to base_link. return angle and distance. '''

    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y
    waypoint_x = waypoint[0]
    waypoint_y = waypoint[1]

    next_distance = math.sqrt((waypoint_x-bot_x)**2 + (waypoint_y-bot_y)**2)

    next_angle = math.atan( (waypoint_x-bot_x)/(waypoint_y-bot_y) ) # returns in RADIANS

    return next_distance, next_angle # angle in RADIANS     


  # def hit_waypoint(odom,path,distance,angle): # UNCOMMENT
  def hit_waypoint(self,odom):
    ''' go towards given waypoint '''
    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y
    bot_heading = convert_pose_to_xy_and_theta(odom)

    path = [(1,1),(2,2),(3,3)] # HARDCODED

    for i in range(0,len(path)):

      print 'going to waypoint ', path[i]

      waypoint_x = path[i][0]
      waypoint_y = path[i][1]

      [next_distance, next_angle] = self.recalculate(odom,path[i]) # angle in RADIANS

      if (waypoint_x > bot_x) & (abs(bot_heading-next_angle) > self.threshold):
        self.spin_left()
      elif (bot_x > waypoint_x) & (abs(bot_heading-next_angle) > self.threshold):
        self.spin_right()
      else:
        self.forward(0.05)

      if (abs(bot_x-waypoint_x) > self.threshold) & (abs(bot_y-waypoint_y) > self.threshold):
        pass
      else:
        self.stop()
        print 'HIT waypoint', path[i]


  def react_scan(self, scan):
    xy_points = []
    for t in range(len(scan.ranges)):
      if t > 0:
        xy_points.append(tr_to_xy((t, scan.ranges[t])))

  def stop(self):
    ''' stop all bot motion '''
    self.command.linear.x = 0
    self.pub.publish(self.command)

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

  def forward(self, x):
    ''' drive bot forward '''
    self.command.linear.x = x
    self.command.linear.y = 0
    self.command.linear.z = 0
    self.command.angular.x = 0
    self.command.angular.y = 0  
    self.command.angular.z = 0

  def drive(self):
    self.pub.publish(self.command)


### GLOBAL METHODS

def clean(num):
  return float(round(num, 1))

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

def convert_pose_to_xy_and_theta(odom):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    # return pose.position.x, pose.position.y, angles[2]
    return angles[2]


controller = Controller()

while not rospy.is_shutdown():
  # controller.update_command()
  controller.drive()

# graph = GraphGenerator((0,0),(1,2)).generate()
# graph.remove_point((0.1,0.3))
# graph.remove_point((0.1,0.1))
# graph.remove_point((0.1,0.2))
# graph.remove_point((0.1,0.4))
# print graph.navigate((0,0),(.5,.5))

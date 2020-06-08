#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2017 Takaki Ueno

# Modified 2019 Adrián Romero
# Released under the MIT license

"""! @package cpp_uav
This module calculates the coverage path given a polygon
"""

# Import python3's print to suppress warning raised by pylint
from __future__ import print_function

# python libraries
from math import tan
import math

import numpy as np

# rospy
import rospy

# messages
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Path
# service
from cpp_uav.srv import Torres16
import os

class PolygonBuilder(object):

    def __init__(self):
        """! Constructor
        """


        # @var is_polygon_drawn
        #  True if polygon is received from mapviz
        self.is_polygon_drawn = False # Need to be true when the topic is received

        # @var server_node
        #  Instance of ros server
        self.server_node = None

        # @var points
        #  Dictionary to store points
        #  - vertices_x: List of x coordinates of vertices
        #  - vertices_y: List of y coordinates of vertices
        #  - start: geometry_msgs/Point object stores info about start point
        #  - waypoints: List of waypoints returned by a coverage path planner
        # vertices_x, vertices_y and start must be initialized via topic subscription
        self.points = {"vertices_x": list(),
                       "vertices_y": list(),
                       "height": 4,
                       "start": None,
                       "waypoints": list()}
        self.points["start"] = Point() 
        self.points["start"].x = 0.0
        self.points["start"].y = -5.0
        self.subpolygons = []
        self.patches = []
        
        # @var footprint_width
        #  footprint_width of the selected antenna
        self.footprint_width = None

        # @var footprint_length
        #  footprint_length of the selected antenna
        self.footprint_length = None

        # @var horizontal_overwrap
        #  horizontal overwrap for the coverage path planning
        self.horizontal_overwrap = None

        # @var vertical_overwrap
        #  vertical overwrap for the coverage path planning
        self.vertical_overwrap = None

        # @var param_std_footprint_width
        #  ros param's id for the footprint_width of the rfid antenna
        self.param_std_footprint_width = 'drone6/footprint_width'

        #width = Alto de la cámara, length = ancho de la cámara.
        if rospy.has_param(self.param_std_footprint_width):
            self.footprint_width = rospy.get_param(self.param_std_footprint_width)
        else:
            rospy.logerr("Error. Param " + self.param_std_footprint_width + " not found")
            self.footprint_width = Float64(2.5) 
        # @var param_std_footprint_length
        #  ros param's id for the footprint_length of the rfid antenna
        self.param_std_footprint_length = 'drone6/footprint_length'

        if rospy.has_param(self.param_std_footprint_length):
            self.footprint_length = rospy.get_param(self.param_std_footprint_length)
        else:
            rospy.logerr("Error. Param " + self.param_std_footprint_length + " not found")
            self.footprint_length = Float64(4)        
        # @var param_std_horizontal_overwrap
        #  ros param's id for the horizontal_overwrap for the coverage path planning
        self.param_std_horizontal_overwrap = 'drone6/horizontal_overwrap'

        if rospy.has_param(self.param_std_horizontal_overwrap):
            self.horizontal_overwrap = rospy.get_param(self.param_std_horizontal_overwrap)
        else:
            rospy.logerr("Error. Param " + self.param_std_horizontal_overwrap + " not found")
            self.horizontal_overwrap = Float64(0.8)        

        # @var param_std_vertical_overwrap
        #  ros param's id for the vertical_overwrap for the coverage path planning
        self.param_std_vertical_overwrap = 'drone6/vertical_overwrap'
        if rospy.has_param(self.param_std_vertical_overwrap):
            self.vertical_overwrap = rospy.get_param(self.param_std_vertical_overwrap)
        else:
            rospy.logerr("Error. Param " + self.param_std_vertical_overwrap + " not found")
            self.vertical_overwrap = Float64(0.8)

        # @var coverage_params
        #  Dictionary of coverage params
        #  - footprint_width [m]: Width of footprint
        #  - footprint_length [m]: Length of footprint
        #  - horizontal_overwrap [%]: Horizontal overwrap of footprint
        #  - vertical_overwrap [%]: Vertical overwrap of footprint
        self.coverage_params = {"footprint_width":
                                self.footprint_width,
                                "footprint_length":
                                    self.footprint_length,
                                "horizontal_overwrap": self.horizontal_overwrap,
                                "vertical_overwrap": self.vertical_overwrap}
        # @var zig_zag_trayectory_publisher
        #  Waypoint trajectory ROS' publisher
        self.zig_zag_trayectory_publisher = rospy.Publisher("drone6/zig_zag_trayectory",PolygonStamped,queue_size=10)

        # @var coverage_path_publisher
        #  Coverage path generated by the algorithm in mapviz format
        self.coverage_path_publisher = rospy.Publisher("drone6/coverage_path",Path,queue_size=10)

        # @var nav_msgs_path
        #  path for mapviz visualization
        self.nav_msgs_path = Path()
        
        # @var PoseStamped
        #  Posestamped object that will be pushed back into nav_msgs_path
        #self.PoseStamped = PoseStamped()

        # @var trayectory 
        #  Polygon with the information of all the calculated waypoints
        self.trayectory = PolygonStamped()

        # @var param_std_altitude
        #  name of the ros param for the altitude of the flying

        self.param_std_altitude = 'drone6/altitude'
        # @const altitude
        #  Float const that determine the altitude of the flying
        if rospy.has_param(self.param_std_altitude):
            self.altitude = rospy.get_param(self.param_std_altitude)
        else:
            self.altitude = 4
            rospy.logerr("Error. Param " + self.param_std_altitude + " not found")
        self.path = os.path.join(os.path.expanduser('~'),'catkin_ws','src','cpp_uav','planes','A400m_cola.txt') 
        print(self.path)

        self.result_path = os.path.join(os.path.expanduser('~'),'catkin_ws','src','cpp_uav','planes','result.txt')
        #"home/adrian/catkin_ws/src/cpp_uav/planes/example.txt"

        self.x_center = None
        self.read_data(self.path)
        self.calculate_path()
        #Once every var is initalized, rospy listener starts
        self.listener()
        
        
    
    def calculate_path(self):
        """!
        Function which calculates the coverage path
        """
        if not self.is_polygon_drawn:
            return

        # assign server node if server node is None
        if not self.server_node:
            rospy.loginfo("Waiting for Server Node.")
            try:
                rospy.wait_for_service("cpp_torres16",
                                       timeout=5.0)
            except rospy.ROSException:
                rospy.logerr("Server not found.")
                return
            try:
                self.server_node = rospy.ServiceProxy(
                    "cpp_torres16",
                    Torres16)
            except rospy.ServiceException as ex:
                rospy.logerr(str(ex))
                return

        # Create a list of vertices
        vertices = []
        waypoint_xs = []
        waypoint_ys = []

        # Fill the list of vertices that is passed to server node
        for x_coord, y_coord in zip(self.points["vertices_x"],self.points["vertices_y"]):
            point = Point()
            point.x = x_coord
            point.y = y_coord
            vertices.append(point)

        # Call service
        try:
            ret = self.server_node(vertices,
                                   self.points["start"],
                                   self.coverage_params["footprint_length"],
                                   self.coverage_params["footprint_width"],
                                   self.coverage_params["horizontal_overwrap"],
                                   self.coverage_params["vertical_overwrap"])

            self.points["waypoints"] = ret.path#geometry_msgs/Point vector (x,y,z)
            print(ret.path)
            self.subpolygons = ret.subpolygons

        except rospy.ServiceException as ex:
            rospy.logerr(str(ex))
            return
        # Publish the calculated path
        cont = 0
        with open(self.result_path,'w') as f:
            for j in range(np.size(ret.path)):

                ### Conversion to mapviz type
                self.PoseStamped = PoseStamped()
                self.PoseStamped.header.stamp = rospy.Time.now()
                self.PoseStamped.header.frame_id = "map"
                self.PoseStamped.header.seq = j
                self.PoseStamped.pose.position.x = ret.path[j].x
                self.PoseStamped.pose.position.y = ret.path[j].y
                self.PoseStamped.pose.position.z = self.altitude
                self.nav_msgs_path.poses.append(self.PoseStamped)
                ###
                f.write(str(cont) + "," + str((ret.path[j].x+self.x_center)*1000) +',0,' + str(ret.path[j].y*1000)+'\n')
                cont += 1
        #print(self.nav_msgs_path.poses)
        print("Finished")
        self.nav_msgs_path.header.stamp = rospy.Time.now()
        self.nav_msgs_path.header.frame_id = "map" 
        self.nav_msgs_path.header.seq = 0
        self.coverage_path_publisher.publish(self.nav_msgs_path)
       ###########################################################
    def listener(self):
    	rospy.Subscriber("/mapviz/ROI", PolygonStamped, self.mapviz_callback)#Los puntos del mapviz son del tipo PolygonStamped

    	rospy.spin()

    def read_data(self,path):
        aux_x = list()
        with open(path) as fp:
            lines = fp.readlines()
            for line in lines:
                point_x, point_y = line.split(',')
                aux_x.append(float(point_x))
                #self.points["vertices_x"].append(float(point_x))
                self.points["vertices_y"].append(float(point_y.split('\n')[0]))
        self.x_center = min(aux_x)
        print(self.x_center)
        self.points["vertices_x"] = [i-self.x_center for i in aux_x]

        print(self.points)
        self.is_polygon_drawn = True

    def mapviz_callback(self,data):
    	self.points["start"] = Point()
    	for i in range(np.size(data.polygon.points)):
    		if i == 0: #First point drawed in mapviz will be considered the start point, for now, in the future used one clicked point for start point
    			self.points["start"].x = data.polygon.points[0].x
    			self.points["start"].y = data.polygon.points[0].y
    		else: #Rest of points are the ones that form the polygon in mapviz
    			self.points["vertices_x"].append(data.polygon.points[i].x)
    			self.points["vertices_y"].append(data.polygon.points[i].y)
    	self.is_polygon_drawn = True
    	self.calculate_path()

def init_node():
    """!
    Initialize a node
    """
    rospy.init_node('utw_node', anonymous=True)

    # Call PolygonBuilder's constructor
    PolygonBuilder()

    
if __name__ == '__main__':
    init_node()
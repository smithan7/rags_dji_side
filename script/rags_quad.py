#!/usr/bin/env python

from utils import *
from mapping import Mapping
from navigation import Navigation, Loc

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg
import dji_sdk.srv

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from dji_sdk.msg import GlobalPosition, TransparentTransmissionData, RCChannels
from custom_messages.msg import Edge_Costs, Nav_Goal, Query_RAGS, Scan_Goals

import time
import sys
import math
import numpy as np

class rags_quad:

	# everything you should need for position, lat lon and local map
	my_lat = -1.0
	my_lon = -1.0

	cLoc = Loc( [0.0, 0.0, 10.0, 0.0] )

	# everythin	g to describe where I am going
	goal_lat = -1.0
	goal_lon = -1.0
	goal_alt = 10.0

	gLoc = Loc( [0.0, 0.0, 10.0, 0.0] )

	# map origin, for converting local to lat/lon 
	origin_lat = -1.0
	origin_lon = -1.0

	RAGS_query_time = 0.0 
	RAGS_query_interval = 3.0

	status_report_time = 0.00
	status_report_interval = 3.0

	my_RAGS_vertex = 0
	goal_RAGS_vertex = -1
	vertices_to_scan_locs = []
	vertices_to_scan_indices = []

	state = "waiting for scan vertices"
	# states for tracking current tasks

	edge_cost_publisher = rospy.Publisher('/edge_costs', Edge_Costs, queue_size=10)
	waiting_on_RAGS_publisher = rospy.Publisher('/waiting_on_RAGS', Query_RAGS, queue_size=10)

	def __init__(self, drone, use_rags):

		if use_rags:
			print("Starting RAGS Node")
		else:
			print("Startin A* Node")

		self.USE_RAGS = use_rags
		
		self.my_mapping = Mapping( 400.0, 400.0, 1.0 )
		self.my_navigation = Navigation( 1.0 ) # max speed of the quad

		# initialize drone
		self.drone = drone
		self.drone.request_sdk_permission_control() #Request to obtain control
		
		rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback_odom)
		rospy.Subscriber("/dji_sdk/global_position", GlobalPosition, self.callback_gps)
		rospy.Subscriber("/dji_sdk/rc_channels", RCChannels, self.callback_rc)

		# from rags algorithm side		
		rospy.Subscriber("/edges_to_scan", Scan_Goals, self.vertices_to_scan_callback)
		rospy.Subscriber("/waypoint_to_travel", Nav_Goal, self.vertex_to_travel_callback)

		#self.drone.landing() # Landing
		#self.drone.release_sdk_permission_control() #Release control

		self.RAGS_query_time = rospy.get_time()

	def publish_edge_costs( self, costs ):
		cost_str = ""
		for cost in costs:
			cost_str += "%0.6f," % cost
		print( "publishing edge costs: %s", cost_str )
		#edge_cost_publisher.publish( cost_str )  convert to Edge_Costs.msg

	def query_RAGS( self ):
		if rospy.get_time() - self.RAGS_query_time > self.RAGS_query_interval:
			self.RAGS_query_time = rospy.get_time()
			query = Query_RAGS()
			query.my_lat = self.my_lat
			query.my_lon = self.my_lon
			query.my_vertex_index = self.my_RAGS_vertex			
			if self.state == "waiting for scan vertices":
				print "Waiting on RAGS vertices to scan"
				query.request_type = 0
				self.waiting_on_RAGS_publisher.publish( query )
			if self.state == "waiting for travel vertex":
				print "Waiting on RAGS goal vertex"
				query.request_type = 1
				self.waiting_on_RAGS_publisher.publish( query )
				
	def action_server( self ):

		if( rospy.get_time() - self.status_report_time > self.status_report_interval ):
			self.status_report_time = rospy.get_time()
			print "in action server at state: ", self.state , ", " , [self.my_lat, self.my_lon, self.goal_lat, self.goal_lon ]

			d = distance_from_a_to_b( self.my_lat, self.my_lon, self.goal_lat, self.goal_lon )
			b = heading_from_a_to_b( self.my_lat, self.my_lon, self.goal_lat, self.goal_lon )

			print "dist to goal: ", d*math.cos( b ), " , ", d*math.sin( b ), " meters"
			
		if self.state == "scanning vertices": # am I scanning edges
			#print "at: ", self.state
			if not self.my_navigation.complete( self.cLoc ):
				[vx, vy, vz, vw] = self.my_navigation.nav( self.cLoc )
				self.drone.velocity_control(1,vx,vy,vz,vw)
			else:
				self.state = "reporting_costs"
				edge_costs = self.my_mapping.estimate_travel_costs( self.cLoc, self.vertices_to_scan_locs )
				self.publish_edge_costs( edge_costs )
				self.state = "waiting for travel vertex"

		elif self.state == "planning travel to vertex":
				[gg_y, gg_x] = GPS_to_local( self.goal_lat, self.goal_lon, self.origin_lat, self.origin_lon)
	
				gLoc = Loc( [gg_x, gg_y, self.goal_alt, heading_from_a_to_b( self.my_lat, self.my_lon, self.goal_lat, self.goal_lon )] )

				travel_path = self.my_mapping.get_path( self.cLoc, gLoc )
				self.my_navigation.set_wp_list( travel_path )
				self.state = "travelling to vertex"

		elif self.state == "travelling to vertex":
			#print "at: ", self.state
			if not self.my_navigation.complete( self.cLoc ):
				[vx, vy, vz, vw] = self.my_navigation.nav( self.cLoc )
				#print "v: ", [vx, vy, vz, vw]
				self.drone.velocity_control(1,vx,vy,vz,vw)				
			else:
				self.state = "waiting for scan vertices"
				print "travelled to vertex # " , self.goal_RAGS_vertex
				self.my_RAGS_vertex = self.goal_RAGS_vertex
				print "waiting at vertex #: ", self.my_RAGS_vertex
			

	def callback_odom(self, data):
		# update my position in the local frame
		#self.my_local_x = data.pose.pose.position.x
		#self.my_local_y = data.pose.pose.position.y
		self.cLoc.z = data.pose.pose.position.z

		# get my heading
		q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
		[self.cLoc.w, trash, garbage] = quaternions_to_RPY( q )

	def callback_gps(self, data):
		if self.origin_lat == -1 and self.origin_lon == -1:
			self.origin_lat = data.latitude
			self.origin_lon = data.longitude
		
		self.my_lat = data.latitude
		self.my_lon = data.longitude
		# move to my local frame!
		[self.cLoc.y, self.cLoc.x] = GPS_to_local( self.my_lat, self.my_lon, self.origin_lat, self.origin_lon)
		
		# call action server
		self.action_server()
		# check in with RAGS alg
		self.query_RAGS()

		#print "loc update: ", [ self.cLoc.x , self.cLoc.y, self.cLoc.z, self.cLoc.w ]
		#print("got lat/lon: ", data.latitude, " / ", data.longitude )
		
	def callback_rc(self, data):
		#print("got rc roll / pitch: ", data.roll, " / ", data.pitch )
		#print("got rc yaw / throttle: ", data.yaw, " / ", data.throttle )
		#print("got rc mode: ", data.mode )
		a = 7.0
	
	def vertex_to_travel_callback( self, vertex ):	
		# this takes in 1 waypoint and converts it to local
		# it is then converted to map and 
		# then uses planner to plan a path to the wp
		# then converts the path back to the local frame
		
		if not self.USE_RAGS and self.state == "waiting for scan vertices":
			# am I using A star skip waiting on scanning edges
			self.state = "waiting for travel vertex"

		if self.state != "waiting for travel vertex":
			print "NOT READY FOR GOAL VERTEX"
			return

		print "vertex_to_travel_callback: ", vertex

		self.goal_lat = vertex.goal_lat
		self.goal_lon = vertex.goal_lon
		self.goal_RAGS_vertex = vertex.goal_index
		
		self.state = "planning travel to vertex"
		
	def vertices_to_scan_callback( self, data ):
		# this takes in a series of lat/lon of adjacent vertices
		# then creates a series of wp at the current location with
		# a bearing that aims the quad at the vertices
		# the action server will then run the quad through orienting at each

		print("wps_to_scan_callback: %s", data)

		if self.state == "waiting for scan vertices":
			self.state = "planning to scan vertice"
		else:
			print("Not ready to recieve vertices to scan")
			return

		if len( data.indices ) == 0:
			print "No edges given to scan, requesting nav goal"
			self.state = "waiting for travel vertex"

		self.vertices_to_scan_locs = []
		for i in range(0,len(data.indices) ):
			bearing = heading_from_a_to_b( self.my_lat, self.my_lon, data.lats[i], data.lons[i] )
			self.vertices_to_scan_locs.append( Loc( [self.my_local_x, self.my_local_y, self.goal_alt, bearing] ) )
			self.vertices_to_scan_indices.append( data.indice[i] )

		self.my_navigation.set_wp_list( self.vertices_to_scan_locs )

		self.state = "scanning vertices"



#!/usr/bin/env python

from utils import *
from fmm import FMM

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from dji_sdk.msg import GlobalPosition
from dji_sdk.msg import TransparentTransmissionData
from dji_sdk.msg import RCChannels
from actionlib_msgs.msg import GoalStatusArray

import time
import sys
import math

class rags_quad:

	# everything you should need for position, lat lon and local map
	my_lat = -1.0
	my_lon = -1.0
	my_local_x = -1.0
	my_local_y = -1.0
	my_alt = -1.0
	my_heading = -1.0

	# list of places to go
	goal_local_list = []

	# everything to describe where I am going
	goal_lat = -1.0
	goal_lon = -1.0
	goal_local_x = -1.0
	goal_local_y = -1.0
	goal_alt = 10.0
	goal_heading = 0.0

	# map origin, for converting local to lat/lon 
	origin_lat = -1.0
	origin_lon = -1.0

	RAGS_query_time = rospy.get_time()
	RAGS_query_interval = 3.0

	state = 0
	# states for tracking current tasks
	# 0 - waiting for vertices to scan
	# 1 - scanning edges
	# 2 - waiting for vertice goal
	# 3 - travelling to vertice goal

	pos_tol = 1.0 # meters
	heading_tol = 5.0 # degrees

	map_resolution = -1.0
	map_height = -1.0
	map_width = -1.0
	map_offset_x = -1.0
	map_offset_y = -1.0

	def __init__(self, use_rags):
		if use_rags:
			print("Starting RAGS Node")
		else:
			print("Startin A* Node")

		self.USE_RAGS = use_rags

		# initialize drone
		self.drone = DJIDrone()
		self.drone.request_sdk_permission_control() #Request to obtain control

		rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback_odom)
		rospy.Subscriber("/dji_sdk/global_position", GlobalPosition, self.callback_gps)
		rospy.Subscriber("/dji_sdk/rc_channels", RCChannels, callback_rc)
		rospy.Subscriber("/map", OccupancyGrid, callback_costmap)
		self.tf_pub = tf.TransformBroadcaster()

		edge_cost_publisher = rospy.Publisher('edge_costs', String, queue_size=10)
		waiting_on_RAGS_publisher('waiting_on_RAGS', String, queue_size=10)

		#self.drone.landing() # Landing
		#self.drone.release_sdk_permission_control() #Release control

	def publish_edge_costs( self, costs ):
		cost_str = ""
		for cost in costs:
			cost_str += "%0.6f," % cost
		print( "publishing edge costs: %s", cost_str )
		edge_cost_publisher.publish( cost_str )

	def query_RAGS( self ):
		if rospy.get_time() - self.RAGS_query_time > self.RAGS_query_interval:
			self.RAGS_query_time = rospy.get_time()
			if self.state == 0:
				print("Waiting on RAGS vertices to scan")
				query_str = "0"
				waiting_on_RAGS_publisher.publish( query_str )
			if self.state == 2:
				print("Waiting on RAGS goal vertex")
				query_str = "1"
				waiting_on_RAGS_publisher.publish( query_str )
				
	def action_server( self ):

		if self.state != 1 or self.state != 3: # if i am not scanning edges or travelling, do nothing
			self.drone.local_position_navigation_cancel_all_goals()
			return

		if self.state == 1: # am I scanning edges
			if at_local_goal():
				self.drone.local_position_navigation_cancel_all_goals()
				time.sleep( self.goal_pause )
				self.goal_local_index += 1
				if len( self.goal_local_list ) > self.goal_local_index:
					# there are still goals, select the next one and move to it
					[self.goal_local_x, self.goal_local_y] = self.goal_local_list[self.goal_local_index]
					if self.safe_goal(): # is it reasonalby close to me?
						self.drone.local_position_navigation_send_request(self.goal_local_x, self.goal_local_y, self.goal_alt, self.goal_heading)
					else:
						print("UNSAFE GOAL for scanning")
				else:
					self.state = 2
					edge_costs = self.fmm.estimate_costs( self.goal_local_list )
					self.publish_edge_costs( edge_costs )

		else if self.state == 3: ## am I travelling to a goal vertex
			if at_local_goal():
				self.drone.local_position_navigation_cancel_all_goals()
				time.sleep( self.goal_pause )
				self.goal_local_index += 1
				if len( self.goal_local_list ) > self.goal_local_index:
					# there are still goals, select the next one and move to it
					[self.goal_local_x, self.goal_local_y] = self.goal_local_list[self.goal_local_index]
					if self.safe_goal(): # is it reasonalby close to me?
						self.drone.local_position_navigation_send_request(self.goal_local_x, self.goal_local_y, self.goal_alt, self.goal_heading)
					else:
						print("UNSAFE GOAL for travelling")
				else:
					self.state = 0

	def callback_odom(self, data):
		# update my position in the local frame
		self.my_local_x = data.pose.pose.position.x
		self.my_local_y = data.pose.pose.position.y
		self.my_alt = data.pose.pose.position.z

		# get my heading
		q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
		[roll, pitch, self.my_heading] = quaternions_to_RPY( q )
		
		# print useful stuff
		#print("got odom position: ", data.pose.pose.position )
		#print("got odom orientation: ", data.pose.pose.orientation )
		#print("roll, pitch, yaw: ", roll, ", ", pitch, ", ", self.my_heading)

		# publish the tf for gmapping
		self.tf_publisher( data.pose.pose )

		# call the action server
		self.action_server()
		self.query_RAGS()

	def tf_publisher( self, pose ):
		# this publishes the tf from the world to the quad for gmapping
		pose.position # x,y,z
		pose.orientation # quaternions: x,y,z,w
		self.tf_pub.sendTransform( (pose.position.x, pose.position.y, pose.position.z),
			(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
			rospy.Time.now(), '/quad', '/map')

	def callback_gps(self, data):
		if self.origin_lat == -1 and self.origin_lon == -1:
			origin_lat = data.latitude
			origin_lon = data.longitude
		# not really used as all commands are in local frame
		self.my_lat = data.latitude
		self.my_lon = data.longitude

		#print("got lat/lon: ", data.latitude, " / ", data.longitude )
		#print("got alt / height: ", data.altitude, " / ", data.height )

	def callback_rc(self, data):
		#print("got rc roll / pitch: ", data.roll, " / ", data.pitch )
		#print("got rc yaw / throttle: ", data.yaw, " / ", data.throttle )
		#print("got rc mode: ", data.mode )

	def wp_to_travel_callback( self, wp ):	
		# this takes in 1 waypoint and converts it to local
		# it is then converted to map and 
		# then uses fmm to plan a path to the wp
		# then converts the path back to the local frame
		
		if not self.USE_RAGS and self.state == 0:
			# am I using A star skip waiting on scanning edges
			self.state = 2

		if self.state != 2:
			print("NOT READY FOR GOAL VERTEX")
			return

		self.state = 3
		print("wp_to_travel_callback: %s", wp)

		[ self.goal_lat, self.goa_lon ] = wp.split(",")

		[gg_x, gg_y] = GPS_to_local( self.goal_lat, self.goal_lon, self.origin_lat, self.origin_lon)

		[m_x, my] = local_to_map( self.my_local_x, self.my_local_y, self.map_offset_x, self.map_offset_y, self.map_resolution )
		[g_x, g_y] = local_to_map( gg_x, gg_y, self.map_offset_x, self.map_offset_y, self.map_resolution )
		m_g_list = self.fmm.get_path( m_x, m_y, g_x, g_y )

		self.goal_local_list = []
		self.goal_local_index = 0
		for g in m_g_list:
			self.goal_local_list.append( map_to_local( g[0], g[1], self.map_resolution) )
		
	def wps_to_scan_callback( self, data ):
		# this takes in a series of lat/lon of adjacent vertices
		# then creates a series of wp at the current location with
		# a bearing that aims the quad at the vertices
		# the action server will then run the quad through orienting at each

		print("wps_to_scan_callback: %s", data)

		if self.state = 0:
			self.state = 1
		else:
			print("Not ready to recieve wps to scan")
			return

		data_split = data.split(";")

		self.goal_local_list = []
		self.goal_local_index = 0
		for ds in data_split:
			[lat, lon] = ds.split(",")
			bearing = heading_from_a_to_b( self.my_lat, self.my_lon, lat, lon )
			goal = [self.my_local_x, self.my_local_y, self.goal_alt, bearing]
			self.goal_local_list.append( goal )


	def safe_goal( self ):
		# map is a square, am I on it?
		if abs( self.goal_local_x )/2 < self.map_width and abs( self.goal_local_y )/2 < self.map_height: # should account for resolution and origin, however, this is mostly a sanity check so ignore
			return True
		else:
			return False

	def at_local_goal( self ):
		d_heading =  abs( self.my_heading - self.goal_heading )
		d_pos = math.sqrt( pow(self.my_local_x - self.goal_local_x, 2) + pow(self.my_local_y - self.goal_local_y, 2) )
		if d_heading < self.heading_tol and d_pos < self.pos_tol:
			return True
		else:
			return False

	def callback_costmap( self, data ):

		# get map metadata
		if self.map_resolution == -1.0:
			self.map_resolution = data.info.resolution
			self.map_height = data.info.height
			self.map_width = data.info.width
			self.map_offset_x = data.data.info.origin.position.x / cost_in.info.resolution
			self.map_offset_y = data.data.info.origin.position.y / cost_in.info.resolution
			print( "map_meta_data")
			print( "	resolution: ". self.map_resolution)
			print( "	width/height: ", self.map_width, self.map_height)
			print( "	origin: ", self.map_offset_x, ", ", self.map_offset_y)
			# initialize the planner
			self.fmm = FMM( 1.0, self.map_resolution, 0.5, 2.0) #max_speed, resolution, crash_radius, inflation_radius

		my_map = np.zeros( self.map_width, self.map_height, int8 )

		# get map	
		it = 0
		for x in range(0,self.map_width):
			for y in range(0,self.map_height): # free
				if data.data[it] < 5 and data.data[it] >= 0:
					myMap[x][y] = 0
				elif data.data[it] == -1: # unknown
					myMap[x][y] = 127
				else: # obstacle
					myMap[x][y] = 255
				it += 1

		self.fmm.updateMap( myMap )

		if self.state == 3: ## if I am travelling to a goal then I care about map updates
			# convert my lat/lon goal to local frame
			[gg_x, gg_y] = GPS_to_local( self.goal_lat, self.goal_lon, self.origin_lat, self.origin_lon)
			# convert local locations to map
			[m_x, my] = local_to_map( self.my_local_x, self.my_local_y, self.map_offset_x, self.map_offset_y, self.map_resolution )
			[g_x, g_y] = local_to_map( gg_x, gg_y, self.map_offset_x, self.map_offset_y, self.map_resolution )
			# get path from fmm in map frame
			m_g_list = self.fmm.findPath( [m_x, m_y], [g_x, g_y] )

			# convert path in map frame to local frame and save as goal list
			self.goal_local_list = []
			self.goal_local_index = 0
			for g in m_g_list:
				self.goal_local_list.append( map_to_local( g[0], g[1], self.map_resolution) )

	def demo_mission(self, data):

		# setup subscribers for these for tasks
		#/dji_sdk/drone_task_action/cancel
		#/dji_sdk/drone_task_action/feedback
		#/dji_sdk/drone_task_action/goal
		#/dji_sdk/drone_task_action/result
		#/dji_sdk/drone_task_action/status

		# for global navigation via gps
		#/dji_sdk/global_position_navigation_action/cancel
		#/dji_sdk/global_position_navigation_action/feedback
		#/dji_sdk/global_position_navigation_action/goal
		#/dji_sdk/global_position_navigation_action/result
		#/dji_sdk/global_position_navigation_action/status

		# for local navigation
		#/dji_sdk/local_position_navigation_action/cancel
		#/dji_sdk/local_position_navigation_action/feedback
		#/dji_sdk/local_position_navigation_action/goal
		#/dji_sdk/local_position_navigation_action/result
		#/dji_sdk/local_position_navigation_action/status

		drone.takeoff() # Takeoff
		time.sleep(5)    
		#drone.landing() # Landing
		#drone.gohome() # Go home
		# drone.attitude_control(0x40, 0, 2, 0, 0) # Attitude control sample

		# local position navigation request is from the home point!!!!
		drone.local_position_navigation_send_request(0, 100, 10) # Local Navi Test 
		time.sleep(15)
		drone.local_position_navigation_send_request(100, 0, 10) # Local Navi Test 
		time.sleep(15)
		drone.local_position_navigation_send_request(0, -100, 10) # Local Navi Test 
		time.sleep(15)
		drone.local_position_navigation_send_request(-100, 0, 10) # Local Navi Test      
		time.sleep(15)
		#drone.global_position_navigation_send_request(22.535, 113.95, 100) # GPS Navi Test 
		#newWaypointList = [ # Waypoint List Navi Test 
		#    dji_sdk.msg.Waypoint(latitude = 22.535, longitude = 113.95, altitude = 100, staytime = 5, heading = 0),
		#    dji_sdk.msg.Waypoint(latitude = 22.535, longitude = 113.96, altitude = 100, staytime = 0, heading = 90),
		#    dji_sdk.msg.Waypoint(latitude = 22.545, longitude = 113.96, altitude = 100, staytime = 4, heading = -90),
		#    dji_sdk.msg.Waypoint(latitude = 22.545, longitude = 113.96, altitude = 10, staytime = 2, heading = 180),
		#    dji_sdk.msg.Waypoint(latitude = 22.525, longitude = 113.93, altitude = 50, staytime = 0, heading = -180)]
		#drone.waypoint_navigation_send_request(newWaypointList)
		time.sleep(15)
		drone.gohome()




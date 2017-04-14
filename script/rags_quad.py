#!/usr/bin/env python

from utils import *

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

	my_lat = -1.0
	my_lon = -1.0
	cLoc = [-1, -1]
	gLoc = [-1, -1]
	waypoints = []
	goal_lat = -1.0
	goal_lon = -1.0
	origin_lat = -1.0
	origin_lon = -1.0

	travelling = False # am I travelling along an edge 
	estimated_costs = False # have I scanned adjacent edges and published updated costs

	def __init__(self, use_rags):
		print("initialized")	

		# initialize drone
		self.drone = DJIDrone()
		self.drone.request_sdk_permission_control() #Request to obtain control

		# hardware 1
		self.map_corners = [-123.148355, 44.634827, -123.141590, 44.631767] 
		self.mat_size = [1140.0, 713.0]

		# hardware 0
		self.map_corners = [-123.147305, 44.633991, -123.145064, 44.632184]
		self.mat_size = [281, 366] 

		if use_rags:
		# spin() simply keeps python from exiting until this node is stopped Navi Test 
			# this node really does 2 things
				# 1 - scan the adjacent edges and report their costs
					# this is handled by adjacent nodes_callback and costmap_callback
				# 2 - fly to the selected goal location
					# this is handled by costmap_callback

			# setup subscribers    
			rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback_odom)
			rospy.Subscriber("/dji_sdk/global_position", GlobalPosition, self.callback_gps)
			#rospy.Subscriber("/dji_sdk/rc_channels", RCChannels, callback_rc)
			#rospy.Subscriber("/map", OccupancyGrid, callback_costmap)
			self.tf_pub = tf.TransformBroadcaster()



		else:
			print "A Star planner executing"

			rospy.Subscriber("/dji_sdk/waypoint_navigation_action/status", GoalStatusArray, self.task_status_callback)
			#rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback_odom)
			#rospy.Subscriber("/dji_sdk/global_position", GlobalPosition, self.callback_gps)
			#self.tf_pub = tf.TransformBroadcaster()


			mat_path = [ [1065.0, 667.0], [977.0, 583.0], [895.0, 556.0], [686.0, 422.0], [576.0, 355.0], [372.0, 339.0], [253.0, 316.0], [176.0,622.0] ]
			newWaypointList = [] # Waypoint List Navi Test 
			for p in mat_path:
				lat_lon = mat_to_GPS( self.mat_size, self.map_corners, p )
				print("lat / lon: ", lat_lon[0] , " , " , lat_lon[1] )
				newWaypointList.append( dji_sdk.msg.Waypoint(latitude = lat_lon[0], longitude = lat_lon[1], altitude = 100, staytime = 5, heading = 0) )

			self.drone.waypoint_navigation_send_request(newWaypointList)
			#self.drone.landing() # Landing
			#self.drone.release_sdk_permission_control() #Release control

	def callback_odom(self, data):
		print("got odom position: ", data.pose.pose.position )
		print("got odom orientation: ", data.pose.pose.orientation )
		q = []
		q.append( data.pose.pose.orientation.x )
		q.append( data.pose.pose.orientation.y )
		q.append( data.pose.pose.orientation.z )
		q.append( data.pose.pose.orientation.w )

		[roll, pitch, self.my_heading] = quaternions_to_RPY( q )
		print("roll, pitch, yaw: ", roll, ", ", pitch, ", ", self.my_heading)
		self.tf_publisher( data.pose.pose )

	def tf_publisher( self, pose ):
		pose.position # x,y,z
		pose.orientation # quaternions: x,y,z,w
		self.tf_pub.sendTransform( (pose.position.x, pose.position.y, pose.position.z),
			(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
			rospy.Time.now(), '/quad', '/map')

	def callback_gps(self, data):
		if self.origin_lat == -1 and self.origin_lon == -1:
			origin_lat = data.latitude
			origin_lon = data.longitude
		self.my_lat = data.latitude
		self.my_lon = data.longitude

		print("got lat/lon: ", data.latitude, " / ", data.longitude )
		print("got alt / height: ", data.altitude, " / ", data.height )

	def callback_rc(self, data):
		print("got rc roll / pitch: ", data.roll, " / ", data.pitch )
		print("got rc yaw / throttle: ", data.yaw, " / ", data.throttle )
		print("got rc mode: ", data.mode )

	def wp_to_travel_callback( self, data ):
		if not travelling and scanned_edges:
			my_goals = []
			#[x, y] = data.split(",")
			goal = data.split(",")
			my_goals.append( goal )
			travelling = True

	def wps_to_scan_callback( self, data ):
		# data = "lat, lon, index; ... for all adjacent nodes
		# my goal is then, from the current node, to estimate the true travel cost to reach each node
		# this is done in two steps, first of which handled in this node
		# 1 - scan the adjacent nodes and call gmapping service to activate step two, also set scanned_edge to True	

		if not self.useRags:
			print("wps_to_scan_callback, but using A*")
			return

		if travelling: # if I am already travelling then ignore
			print("wps_to_scan_callback, but travelling")			
			return

		print("wps_to_scan_callback: %s", data)

		data_split = data.split(";")

		self.waypoints = []
		for ds in data_split:
			wp = []
			[x_s, y_s] = ds.split(",")
			x = (x_s)
			wp.append(x)
			y = float(y_s)
			print("x,y: %0.6f, %0.6f",x,y)			
			wp.append( y )
			wp = mat_to_gps( wp )
			print("lon, lat: %0.6f, %0.6f", wp[0], wp[1])			
			self.waypoints.append( wp )

		newWaypointList = [] # Waypoint List Navi Test 
		for wp in self.waypoints:
			lat_lon = self.mat_to_map( p )
			print("lat / lon: ", c_lat , " , " , c_lon )
			bearing = heading_from_a_to_b( self.my_lat, self.my_lon, wp[0], wp[1] )
			newWaypointList.append( dji_sdk.msg.Waypoint(latitude = c_lat_lon[0], longitude = c_lat_lon[1], altitude = 100, staytime = 1, heading = bearing) )

		self.drone.waypoint_navigation_send_request(newWaypointList)
		self.estimated_costs = False

	def task_status_callback( self, data):

		for d in data.status_list:
			print d
			print("l")
		
		#if data.status_list.status != 3:
		#	print("task status callback: still in motion")			
		#	return
		
		if self.estimated_costs:
			print("task status callback: costs are estimated")			
			return

				
		# get actual costs
		costs = []		
		for wp in self.waypoints:
			pt = GPS_to_ground( wp[0], wp[1], origin_lati, origin_longti)
			cost = fm2Star( cLoc, pt )
			costs.append( cost )
		
		self.publish_costs( costs )
		self.estimated_costs = True
	
	def callback_costmap( self, data ):
		# this is the second step in estimating the edge costs
		# now I should have a map, inflate obstacles, perform A* to each goal, report costs

		if travelling == True: # update costmap because I am travelling
			print("callback_costmap while travelling")
		elif scanned_edges == False: # have not scanned edges then don't worry about map
			print("callback_costmap with unscanned edges")
			return

		# get map metadata
		resolution = data.info.resolution
		height = data.info.height
		width = data.info.width
		origin = data.data.info.origin

		my_map = np.zeros( height, width, int8 )

		# get map	
		it = 0
		for x in range(0,width):
			for y in range(0,height):
				if data.data[it] < 5 and data.data[it] >= 0:
					myMap[x][y] = 0
				elif data.data[it] == -1:
					myMap[x][y] = 127
				else:
					myMap[x][y] = 255
				it += 1

		# inflate obstacles
		my_map = inflate_map( my_map, 1 / resolution )

		[my_x, my_y] = GPS_to_ground( my_lat, my_lon )
		[my_xi, my_yi] = ground_to_map( my_x, my_y )

		global origin_lat, origin_lon, my_goals
		costs_str = ""

		if travelling:
			[g_x, g_y] = GPS_to_ground( goal[0], goal[1], origin_lat, origin_lon )
			[g_xi, g_yi] = ground_to_map( g_x, g_y )
			[path, cost] = aStar(my_map, my_xi, my_yi, g_xi, g_yi)
		
			for p in path:
				# publish nav goals to the dji
				[px, py] = map_to_ground( p )


		for goal in my_goals:
			[g_x, g_y] = GPS_to_ground( goal[0], goal[1], origin_lat, origin_lon )
			[g_xi, g_yi] = ground_to_map( g_x, g_y )
			[path, cost] = aStar(my_map, my_xi, my_yi, g_xi, g_yi)
			costs_str +=  "%0.6f, %0.6f, %d, %0.2f;", goal[0], goal[1], goal[2], cost*resolution

		cost_pub.publish(costs_str)


	def aStar( self, my_map, xa, ya, xb, yb ):

		cost = len(path)
		return [path, cost]

	def inflate_map( self, my_map, dist ):
	
		for it in range(0, dist): # number of iterations to inflate
			# find cells to be inflated
			for x in range(1, my_map.shape[0]-1):
				for y in range(1, my_map.shape[1]-1):
					if my_map[x][y] == 255:
						if my_map[x-1][y] <= 127:
							my_map[x-1][y] = 200
						if my_map[x+1][y] <= 127:
							my_map[x+1][y] = 200
						if my_map[x][y-1] <= 127:
							my_map[x][y-1] = 200
						if my_map[x][y+1] <= 127:
							my_map[x][y+1] = 200
		
			# inflate cells	
			for x in range(1, my_map.shape[0]-1):
				for y in range(1, my_map.shape[1]-1):
					if my_map[x][y] == 200:
						my_map[x][y] = 255	

		return my_map


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




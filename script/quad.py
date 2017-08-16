#!/usr/bin/env python

from utils import *
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

# my functions
from custom_rags_messages.msg import DJI_Bridge_Status_MSG, DJI_Bridge_Travel_Path_MSG, DJI_Bridge_Travel_Speed_MSG
from utils import Loc, GPS

import time
import sys
import math
import numpy as np

class rags_quad:

	# everything you should need for position, lat lon and local map
	current_location = Loc( [0.0, 0.0, 10.0, 0.0] ) # lat, lon, alt, heading
	goal_location = Loc( [0.0, 0.0, 10.0, 0.0] ) # lat, lon, alt, heading
	published_goal_location = Loc( [0.0, 0.0, 10.0, 0.0] ) # lat, lon, alt, heading

	status_report_time = 0.00 # this prints
	status_report_interval = 3.0

	status_publisher_time = 0.00 # publish status to other nodes
	status_publisher_interval = 3.0

	heartbeat_time = 0.0 # how long since I heard from planner
	max_heartbeat_interval = 2.0

	get_dji_control_time = 0.0 # how often do I attempt to take control of the quad
	get_dji_control_interval = 5.0

	travel_speed = 0.0

	status_publisher = rospy.Publisher('/dji_bridge_status', DJI_Bridge_Status_MSG, queue_size=10)
	odom_publisher = rospy.Publisher('/dji_bridge_odom', Odom, queue_size=10)

	def __init__(self, drone, nw_corner, se_corner):

		self.navigation = Navigation( nw_corner, se_corner )

		# initialize drone, test 1,2 of 6 for test mode
		self.drone = drone
		self.drone.request_sdk_permission_control() #Request to obtain control
		
		# get true odom from the quad
		rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback_odom)
		rospy.Subscriber("/dji_sdk/global_position", GlobalPosition, self.callback_gps)

		# from algorithm side		
		rospy.Subscriber("/safe_travel_speed", DJI_Bridge_Travel_Speed_MSG, self.safe_travel_speed_callback)
		rospy.Subscriber("/travel_path", DJI_Bridge_Travel_Path_MSG, self.travel_path_callback)

		self.RAGS_query_time = rospy.get_time()

	def publish_status_msg( self ):
		if rospy.get_time() - self.status_publisher_time > self.status_publisher_interval:
			self.status_publisher_time = rospy.get_time()
			report = DJI_Bridge_Status_MSG()
			
			report.longitude = self.current_location.longitude # x loc
			report.latitude = self.current_location.latitude # y loc
			report.altitude = self.current_location.altitude # z loc
			report.heading = self.current_location.heading # heading
			report.goal_longitude = self.goal_location.longitude # goal location ~ travel_path[-1]
			report.goal_latitude = self.goal_location.latitude # goal location

			if self.state == "waiting for travel path":
				report.headingaiting = True
				report.travelling = False
			elif self.state == "travelling":
				report.headingaiting = False
				report.travelling = True

			if self.state == "in emergency stop":
				report.emergency_stopped = True
			else:
				report.emergency_stopped = False

			self.status_publisher.publish( report )

	def safe_travel_speed_callback( self, travel_speed_msg ):
		# this sets the 
		if( travel_speed_msg.stop ):
			### flag to stop was thrown, IMMEDIATELY STOP!
			self.safe_travel_speed = 0.0
			self.drone.velocity_control(1,0.0,0.0,0.0,0.0)
			self.state = "in emergency stop"
		else:
			# set the travel speed
			self.navigation.set_travel_speed( travel_speed_msg.travel_speed )
			# update the time since I heard from planner
			self.heartbeat_time = rospy.get_time()
			if self.state == "in emergency stop":
				self.state = "waiting for travel path"


	def travel_path_callback( self, travel_path_msg ):

		if len(travel_path_msg.longitudes) > 0:
			# this takes in a sequence of Locs (x,y,z,w) and saves them as the current path
			path = []
			for i in range(0,len(travel_path_msg.longitudes)):
				wp = Loc(travel_path_msg.longitudes[i], travel_path_msg.latitudes[i], travel_path_msg.altitudes[i], travel_path_msg.headings[i])
				path.append(wp)

			self.goal_location = path[-1]
			self.navigation.set_wp_list( path )
			# only switch to travelling mode if emergency stop is off
			if self.state != "in emergency stop":
				self.state = "travelling"

	def print_status_report( self ):
		if( rospy.get_time() - self.status_report_time > self.status_report_interval ):
			self.status_report_time = rospy.get_time()
			print("in action server at state: ", self.state , ", " , self.current_location) 
			if self.state == "travelling":
				print("dist to goal: ", self.goal_location.longitude - self.current_location.longitude, " , ", self.goal_location.latitude - self.current_location.latitude, " degrees")

	def action_server( self ):
		if( rospy.get_time() - self.heartbeat_time > self.max_heartbeat_interval ):
			# haven't heard from planner in a while, stop moving!
			print("ERROR::action_server:: no heartbeat from planner")
			self.drone.velocity_control(1,0.0,0.0,0.0,0.0)
			return

		if self.state == "travelling":
			# I am actively travelling along my path
			if not self.my_navigation.complete():
				# I am not at the end of my path, keep planning
				[vx, vy, vz, vw] = self.my_navigation.nav()
				#print "action server::travelling:: v: ", [vx, vy, vz, vw]
				self.drone.velocity_control(1,vx,vy,vz,vw)
			else:
				# I have finished travelling, stop and share
				self.state = "waiting for travel path"
				self.drone.velocity_control(1,0.0,0.0,0.0,0.0)	
			return

		if self.state == "waiting for travel path": ## am I waiting
			# yes, try not to move
			self.drone.velocity_control(1,0.0,0.0,0.0,0.0)	
			return


	def callback_odom(self, data):
		# set my altitude
		self.current_location.altitude = data.pose.pose.position.altitude
		self.navigation.update_alt(self.current_location.altitude)
		
		# get my heading
		q = [data.pose.pose.orientation.longitude, data.pose.pose.orientation.latitude, data.pose.pose.orientation.altitude, data.pose.pose.orientation.heading ]
		[self.current_location.heading, trash, garbage] = quaternions_to_RPY( q ) # all I care about is yaw
		self.navigation.update_heading(self.current_location.heading)
		# update odom info
		
		# publish my own odom topic
		if self.gps_up_to_date == True:
			self.gps_up_to_date = False
			[local_x, local_y] = self.navigation.global_to_local(self.location)
			data.pose.pose.position.x = local_x
			data.pose.pose.position.y = local_y
			self.odom_publisher.publish( data )

	def callback_gps(self, data):
		### this constantly asks for control, if the human user allows it then it is given
		if rospy.get_time() - self.get_dji_control_time > self.get_dji_control_interval:
			self.get_dji_control_time = rospy.get_time()
			self.drone.request_sdk_permission_control() #Request to obtain control
		

		# update location in navigation
		self.navigation.update_location(data.latitude, data.longitude)
		self.current_location.longitude = data.longitude
		self.current_location.latitude = data.latitude

		self.gps_up_to_date = True
		# call action server
		self.action_server()
		# print a status report
		self.print_status_report()
		# publish status report
		self.publish_status_msg()


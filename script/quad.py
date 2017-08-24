#!/usr/bin/env python

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
from custom_messages.msg import DJI_Bridge_Status_MSG, DJI_Bridge_Travel_Path_MSG, DJI_Bridge_Travel_Speed_MSG
from utils import *
from navigation import Navigation


import time
import sys
import math
import numpy as np

class quad:
	
	# everything you should need for position, lat lon and local map
	current_global_location = Global_Loc(0.0, 0.0) # lat, lon
	goal_global_location = Global_Loc(0.0, 0.0) # lat, lon
	published_goal_global_location = Global_Loc(0.0, 0.0) # lat, lon
	
	current_local_location = Local_Loc(0.0, 0.0, 0.0, 0.0) # x,y,alt,heading
	goal_local_location = Local_Loc(0.0, 0.0, 0.0, 0.0) # x,y,alt,heading
	published_local_location = Local_Loc(0.0, 0.0, 0.0, 0.0) # x,y,alt,heading

	status_report_time = 0.00 # this prints
	status_report_interval = 3.0

	status_publisher_time = 0.00 # publish status to other nodes
	status_publisher_interval = 0.1

	heartbeat_time = 0.0 # how long since I heard from planner
	max_heartbeat_interval = 2.0

	get_dji_control_time = 0.0 # how often do I attempt to take control of the quad
	get_dji_control_interval = 5.0

	travel_speed = 0.0
	gps_up_to_date = False

	state = "waiting for travel path"

	status_publisher = rospy.Publisher('/dji_bridge_status', DJI_Bridge_Status_MSG, queue_size=10)
	odom_publisher = rospy.Publisher('/dji_bridge_odom', Odometry, queue_size=10)

	def __init__(self, drone, nw, se):
	#def __init__(self, nw, se):
		nw_corner = Global_Loc(nw[1], nw[0])
		se_corner = Global_Loc(se[1], se[0])
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
		print("DJI_Bridge::Quad Initialized")

	def publish_status_msg( self ):
		if rospy.get_time() - self.status_publisher_time > self.status_publisher_interval:
			self.status_publisher_time = rospy.get_time()
			report = DJI_Bridge_Status_MSG()
			
			report.longitude = self.current_global_location.longitude # x loc
			report.latitude = self.current_global_location.latitude # y loc
			report.altitude = self.current_local_location.altitude # z loc
			report.heading = self.current_local_location.heading # heading
			report.goal_longitude = self.goal_global_location.longitude # goal location ~ travel_path[-1]
			report.goal_latitude = self.goal_global_location.latitude # goal location
			report.local_x = self.current_local_location.local_x
			report.local_y = self.current_local_location.local_y
			report.local_goal_x = self.goal_local_location.local_x
			report.local_goal_y = self.goal_local_location.local_y

			if self.state == "waiting for travel path":
				report.waiting = True
				report.travelling = False
			elif self.state == "travelling":
				report.waiting = False
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

		if len(travel_path_msg.local_xs) > 0:
			# this takes in a sequence of Locs (x,y,z,w) and saves them as the current path
			path = []
			for i in range(0,len(travel_path_msg.local_xs)):
				wp = Local_Loc(travel_path_msg.local_xs[i], travel_path_msg.local_ys[i], travel_path_msg.altitudes[i], travel_path_msg.headings[i])
				path.append(wp)

			self.goal_local_location = path[-1]
			self.navigation.set_wp_list( path )
			# only switch to travelling mode if emergency stop is off
			if self.state != "in emergency stop":
				self.state = "travelling"

	def print_status_report( self ):
		if( rospy.get_time() - self.status_report_time > self.status_report_interval ):
			self.status_report_time = rospy.get_time()
			print "DJI_Bridge::in action server at state: ", self.state , ", " , self.current_global_location.longitude, " / ", self.current_global_location.latitude, ", ", self.current_local_location.local_x, " / ", self.current_local_location.local_y
			if self.state == "travelling":
				print("dist to goal: ", abs(self.goal_local_location.local_x - self.current_local_location.local_x), " , ", abs(self.goal_local_location.local_y - self.current_local_location.local_y), " (m)")

	def action_server( self ):
		#if( rospy.get_time() - self.heartbeat_time > self.max_heartbeat_interval ):
		#	self.heartbeat_time = rospy.get_time()
		#	# haven't heard from planner in a while, stop moving!
		#	print("DJI_Bridge::action_server:: no heartbeat from planner")
		#	self.drone.velocity_control(1,0.0,0.0,0.0,0.0)
		#	return

		if self.state == "travelling":
			# I am actively travelling along my path
			if not self.navigation.complete():
				# I am not at the end of my path, keep planning
				[vx, vy, vz, vw] = self.navigation.nav()
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
		self.current_local_location.altitude = data.pose.pose.position.z
		self.navigation.update_alt(self.current_local_location.altitude)
		
		# get my heading
		q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
		[self.current_local_location.heading, trash, garbage] = quaternions_to_RPY( q ) # all I care about is yaw
		self.navigation.update_heading(self.current_local_location.heading)
		# update odom info
		
		# publish my own odom topic
		if self.gps_up_to_date == True:
			self.gps_up_to_date = False
			data.header.frame_id = "/map"
			data.pose.pose.position.x = self.current_local_location.local_x
			data.pose.pose.position.y = self.current_local_location.local_y
			data.pose.pose.orientation.z = -1*data.pose.pose.orientation.z;			
			self.odom_publisher.publish( data )

	def callback_gps(self, data):
		### this constantly asks for control, if the human user allows it then it is given
		if rospy.get_time() - self.get_dji_control_time > self.get_dji_control_interval:
			self.get_dji_control_time = rospy.get_time()
			self.drone.request_sdk_permission_control() #Request to obtain control
		

		# update location in navigation
		[self.current_local_location.local_x, self.current_local_location.local_y] = self.navigation.update_location(data)
		self.current_global_location.longitude = data.longitude
		self.current_global_location.latitude = data.latitude

		self.gps_up_to_date = True
		# call action server
		self.action_server()
		# print a status report
		self.print_status_report()
		# publish status report
		self.publish_status_msg()



#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import time

from utils import Loc, GPS

class Navigation(object):
	"""description of class"""

	max_speed = 0.0 # doesn't matter set in init	
	max_rate = 15.0
	wp_list = []
	wp_yaw_thresh = 0.175
	wp_dist_3_thresh = 5.0

	kp_xy = 1.0
	kd_xy = 0.0
	kp_z = 1.0
	kd_z = 5.0
	kp_w = 200.0
	kd_w = 100.0

	ex = 0.0
	ey = 0.0
	ez = 0.0
	ew = 0.0

	width_deg = 0.0
	height_deg = 0.0
	width_meters = 0.0
	height_meters = 0.0
	meters_per_cell.x = 0.0
	meters_per_cell.y = 0.0

	def __init__( self, ne_in, sw_in ):
		self.ne_corner = ne_in
		self.sw_corner = sw_in

		self.width_deg = ne_in.lon - sw_in.lon
		self.height_deg = ne_in.lat - sw_in.lat

		[self.width_meters, self.height_meters] = GPS_to_Local(nw_in.y, nw_in.x, se_in.y, se_in.x)
		self.width_meters = math.abs(self.width_meters)
		self.height_meters = math.abs(self.height_meters)

		self.meters_per_deg.x = self.width_meters / self.width_deg
		self.meters_per_deg.y = self.height_meters / self.height_deg

	def global_to_local(self, g):
		# convert from gps to local frame
		px = (g.longitude - self.sw_corner.longitude) / self.width_deg
		py = (g.latitude - self.sw_corner.latitude) / self.height_deg

		l.x = px*self.meters_per_deg.x
		l.y = py*self.meters_per_deg.y
		return l

	def update_location(self, l):
		# in from quad
		[self.cLoc.x, self.cLoc.y ]  = self.global_to_local(l)

	def update_alt( self, cz ):
		# in from quad
		self.cLoc.z = cz

	def update_heading(self, ch ):
		# in from quad
		self.cLoc.w = ch

	def get_local_loc():
		# send back to quad
		return ([self.cLoc.x, self.cLoc.y])

	def constrain_yaw(self, y ):
		pi = 3.141592654		
		if y < 0.0:
			y += 2.0*pi
		if y > 2.0 * pi:
			y -= 2.0*pi
		return y

	def set_travel_speed( self, ts ):
		self.max_speed = ts

	def set_vel( self, g ):
 		
		pi = 3.141592654

		dx = g.x - self.cLoc.x
		dy = g.y - self.cLoc.y
		dz = g.z - self.cLoc.z

		vx = self.kp_xy*(dx) + self.kd_xy*(dx - self.ex)
		vy = -1.0 * ( self.kp_xy*( dy ) + self.kd_xy*(dy - self.ey) )
		vz = self.kp_z*( dz) + self.kd_z*(dz - self.ez)
		
		self.ex = dx
		self.ey = dy
		self.ez - dz

		yaw = math.atan2( self.ey, -self.ex ) + pi
		#print "yaw: ", yaw*180.0/pi

		# dji works 0 -> 2 pi
		yaw = self.constrain_yaw( yaw )
		my_heading_constrained = self.constrain_yaw( c.w )		
		#print "yaw constrained: ", yaw*180.0/pi

		# rollover problem
		if yaw > c.w + pi:
		#	print "threshold: ", (c.w+pi)*180.0/pi
			yaw -= 2*pi

		#print "yaw with rolloever fixed: ", yaw*180.0/pi
		#print "my heading: ", c.w*180.0/pi

		vw = self.kp_w*( yaw - c.w ) + self.kd_w*(yaw - c.w - self.ew)		
		self.ew = yaw - c.w

		# pointed the right way at the right alt before moving fast
		if abs( self.ew ) > pi / 2 or abs( self.ez ) > 5: 
			vx = max(-self.max_speed/4.0, min( vx, self.max_speed/4.0 ) )
			vy = max(-self.max_speed/4.0, min( vy, self.max_speed/4.0 ) )
			vz = max(-self.max_speed, min( vz, self.max_speed ) ) 
			vw = max(-self.max_rate, min( vw, self.max_rate ) )
		elif abs( self.ew ) > pi / 4 or abs( self.ez ) > 2: 
			vx = max(-self.max_speed/2.0, min( vx, self.max_speed/2.0 ) )
			vy = max(-self.max_speed/2.0, min( vy, self.max_speed/2.0 ) )
			vz = max(-self.max_speed, min( vz, self.max_speed ) ) 
			vw = max(-self.max_rate, min( vw, self.max_rate ) )
		else:
			vx = max(-self.max_speed, min( vx, self.max_speed ) )
			vy = max(-self.max_speed, min( vy, self.max_speed ) )
			vz = max(-self.max_speed, min( vz, self.max_speed ) ) 
			vw = max(-self.max_rate, min( vw, self.max_rate ) )
			#print "v: ", [vx, vy, vz]

		if len( self.wp_list ) == 1:
			d = math.sqrt( pow( c.x-g.x,2) + pow( c.y-g.y,2) )
			if d < 2*self.max_speed:
				vx *= d / (2*self.max_speed)
				vy *= d / (2*self.max_speed)
		
		return [ vx, vy, vz, vw]		

	def complete( self ):
		self.get_next_wp()
		if len( self.wp_list ) == 0:
			return True
		else:
			return False

	def set_wp_list( self, wpl ):
		# in from quad
		self.wp_list = wpl

	def nav( self ):
		# am I at my goal
		if self.at_point_xyzw( self.wp_list[ 0 ] ):
			self.get_next_wp()
		
		# set vels
		if len( self.wp_list ) >= 1:
			#print "setting nav vels" 
			vels = self.set_vel( self.cLoc, self.wp_list[ 0 ] )
			return vels
		else:
			vels = [0.0,0.0,0.0,0.0]
			return vels

	def get_next_wp( self ):
		# always navigate to first wp, so check through list until I find a point I am not at yet		
		while len( self.wp_list ) > 0 and self.at_point_xyz( self.cLoc, self.wp_list[0] ):
			del self.wp_list[0]

	def get_current_wp( self ):
		wp_i = len(self.wp_list)-1
		while wp_i > 0:
			if self.at_point_xyz(self.wp_list[wp_i]):
				return wp_i + 1
			wp_i = wp_i - 1

	def at_point_xy( self, g ):
		# get distance between two points
		d = euclid2d(self.cLoc, g)
		# am I close enough to be done
		if d < self.wp_dist_3_thresh:
			return True
		else:
			return False

	def at_point_xyz( self, g ):
		# get distance between some lat and lon
		d = euclid3d(self.cLoc, g)# am I close enough to be done
		if d < self.wp_dist_3_thresh:
			return True
		else:
			return False

	def at_point_xyzw( self, g ):
		# get distance between some lat and lon
		d = euclid3d(self.cLoc, g)
		# am I close enough to be done
		if d < self.wp_dist_3_thresh and abs(self.ew) < self.wp_yaw_thresh:
			return True
		else:
			return False
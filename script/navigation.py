import numpy as np
import cv2
import math
import time
import rospy

class Navigation(object):
	"""description of class"""

	max_speed = 10.0 # doesn't matter set in init	
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

	def __init__( self, max_speed ):
		self.max_speed = max_speed

	def constrain_yaw(self, y ):
		pi = 3.141592654		
		if y < 0.0:
			y += 2.0*pi
		if y > 2.0 * pi:
			y -= 2.0*pi
		return y

	def set_vel( self, c, g ):
 		
		pi = 3.141592654

		vx = self.kp_xy*(g.x - c.x) + self.kd_xy*(g.x - c.x - self.ex)
		vy = -1.0 * ( self.kp_xy*( g.y - c.y ) + self.kd_xy*(g.y - c.y - self.ey) )
		vz = self.kp_z*( g.z-c.z ) + self.kd_z*(g.z - c.z - self.ez)
		
		self.ex = g.x - c.x
		self.ey = g.y - c.y
		self.ez - g.z - c.z

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

	def set_scan_vel( self, c, g ):
 		
		pi = 3.141592654

		vz = self.kp_z*( g.z-c.z ) + self.kd_z*(g.z - c.z - self.ez)
		
		self.ex = g.x - c.x
		self.ey = g.y - c.y
		self.ez - g.z - c.z

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
		
		return [ 0.0, 0.0, vz, vw]

	def complete( self, c ):
		self.get_next_wp( c )
		if len( self.wp_list ) == 0:
			return True
		else:
			return False


	def set_wp_list( self, wpl ):
		self.wp_list = wpl

	def nav( self, c ):
		self.cLoc = c
		if self.at_point( self.cLoc, self.wp_list[ 0 ] ):
			self.get_next_wp( self.cLoc )
		
		vels = []
		if len( self.wp_list ) >= 1:
			#print "setting nav vels" 
			vels = self.set_vel( self.cLoc, self.wp_list[ 0 ] )
		else:
			vels = [0.0,0.0,0.0,0.0]

		return vels

	def scan( self, c ):
		self.cLoc = c
		
		if self.at_scan_angle():
			self.get_next_scan_goal()
		
		vels = []
		if len( self.wp_list ) >= 1:
			vels = self.set_scan_vel( self.cLoc, self.wp_list[ 0 ] )
		else:
			vels = [0.0,0.0,0.0,0.0]

		return vels

	def get_next_scan_goal( self ):
		# always navigate to first wp, so check through list until I find a point I am not at yet		
		while len( self.wp_list ) > 0 and self.at_scan_angle():
			del self.wp_list[0]

	def get_next_wp( self, c ):
		# always navigate to first wp, so check through list until I find a point I am not at yet		
		while len( self.wp_list ) > 0 and self.at_point( c, self.wp_list[0] ):
			del self.wp_list[0]

	def at_scan_angle( self ):
		if abs(self.ew) < self.wp_yaw_thresh:
			return True
		else:
			return False

	def at_point( self, c, g ):
		d = math.sqrt( pow( c.x-g.x,2) + pow( c.y-g.y,2) + pow( c.z-g.z,2) )
		
		if d < self.wp_dist_3_thresh and abs(self.ew) < self.wp_yaw_thresh:
			return True
		else:
			return False

	
class Loc(object):
	def __init__( self, arg ):
		self.x = arg[0]
		self.y = arg[1]
		self.z = arg[2]
		self.w = arg[3]





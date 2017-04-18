import numpy as np
import cv2
import math
import time
import rospy

class Navigation(object):
	"""description of class"""

	max_speed = 1.0	
	max_rate = 20.0
	wp_list = []
	wp_yaw_thresh = 10.0
	wp_dist_3_thresh = 5.0

	ex = 0.0
	ey = 0.0
	ez = 0.0
	ew = 0.0

	def __init__( self, max_speed ):
		self.max_speed = max_speed

	def set_intermediate_vel( self, c, g ):
 		
		pi = 3.141592654

		vx = 0.5*(g.x - c.x) + 0.1*(g.x - c.x - self.ex)
		vy = -0.5*( g.y - c.y ) - 0.1*(g.y - c.y - self.ey)
		vz = 2.0*( g.z-c.z ) + 0.1*(g.z - c.z - self.ez)
		
		self.ex = g.x - c.x
		self.ey = g.y - c.y
		self.ez - g.z - c.z

		yaw = math.atan2( self.ey, -self.ex ) + pi
		# dji works 0 -> 2 pi
		if yaw < 0.0:
			yaw += 2.0*pi
		# rollover problem
		
		if yaw > 0.0 and yaw < pi:
			if abs( yaw + 2*pi - c.w ) < abs( yaw - c.w ):	
				yaw += 2.0*pi
		else:
			if abs( yaw - 2*pi - c.w ) < abs( yaw - c.w ):	
				yaw -= 2.0*pi

		vw = 200*( yaw - c.w ) + 0.1*(yaw - c.w - self.ew)		
		self.ew = yaw - c.w

		# pointed the right way at the right alt before moving fast
		if abs( self.ew ) > pi / 2 or abs( self.ez ) > 5: 
			vx = max(-self.max_speed/4.0, min( vx, self.max_speed/4.0 ) )
			vy = max(-self.max_speed/4.0, min( vy, self.max_speed/4.0 ) )
			vz = max(-self.max_speed*2.0, min( vz, self.max_speed*2.0 ) ) 
			vw = max(-2*self.max_rate, min( vw, 2*self.max_rate ) )
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

		return [ vx, vy, vz, vw]
		
	def set_final_vel( self, c, g ):
		d = math.sqrt( pow( c.x-g.x,2) + pow( c.y-g.y,2) )
		vx = 0.5*( g.x - c.x ) / d * self.max_speed
		vy = -0.5*( g.y - c.y ) / d * self.max_speed
		vz = 2*( g.z-c.z )

		#print "set_vel_x: 0.5*(", c.x, " - ", g.x, ") / " , d, " * ", self.max_speed, " = ", vx
		#print "set_vel_y: 0.5*(", g.y, " - ", c.y, ") / " , d, " * ", self.max_speed, " = ", vy

		vw = 10*( g.w - c.w )

		vx = max(-self.max_speed, min( vx, self.max_speed ) )
		vy = max(-self.max_speed, min( vy, self.max_speed ) )
		vz = max(-self.max_speed, min( vz, self.max_speed ) ) 

		if d < 2*self.max_speed:
			vx *= d / (2*self.max_speed)
			vy *= d / (2*self.max_speed)

		return [vx, vy, vz, vw]

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
		if len( self.wp_list ) == 1:
			vels = self.set_final_vel( self.cLoc, self.wp_list[ 0 ] )
		elif len( self.wp_list ) > 1:
			vels = self.set_intermediate_vel( self.cLoc, self.wp_list[ 0 ] )
		else:
			vels = [0.0,0.0,0.0,0.0]

		return vels

	def get_next_wp( self, c ):
		# always navigate to first wp, so check through list until I find a point I am not at yet		
		while len( self.wp_list ) > 0 and self.at_point( c, self.wp_list[0] ):
			del self.wp_list[0]

	def at_point( self, c, g ):
		d = math.sqrt( pow( c.x-g.x,2) + pow( c.y-g.y,2) + pow( c.z-g.z,2) )
		yd = c.w - g.w

		if d < self.wp_dist_3_thresh and yd < self.wp_yaw_thresh:
			return True
		else:
			return False

	
class Loc(object):
	def __init__( self, arg ):
		self.x = arg[0]
		self.y = arg[1]
		self.z = arg[2]
		self.w = arg[3]





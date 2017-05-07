import numpy as np
import cv2
import math
import time

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from utils import *
from State import State
from navigation import Loc

class Mapping(object):

	odom_x = -1.0
	odom_y = -1.0
	odom_z = 0.0

	map_x = -1
	map_y = -1

	odom_init = False
	scan_init = False

	obstacle = 0
	free = 255
	unknown = 127

	cellsPerMeter = 1.0
	obstacle_radius = 2.0

	display_map_timer = 0.0
	display_map_interval = 1.0
	waiting_on_odom_timer = 0.0
	waiting_on_odom_interval = 1.0

	min_scan_dist = 0.0
	max_scan_dist = 0.0
	
	def __init__( self, width, height, cellsPerMeter ):

		self.my_map = self.unknown * np.ones( (int(width * cellsPerMeter), int(height * cellsPerMeter) ), np.dtype(np.uint8) )
		self.occ = 0.5 * np.ones( np.shape( self.my_map ) )
		self.width = width * cellsPerMeter
		self.height = height * cellsPerMeter
		self.cellsPerMeter = cellsPerMeter
		self.origin_x = self.width / 2
		self.origin_y = self.height / 2

		rospy.Subscriber("/scan", LaserScan, self.callback_scan)
		rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback_odom)

	def callback_odom(self, data):
		self.odom_z = data.pose.pose.position.z
		if self.odom_z < 2.0:
			self.odom_init = False
			return			

		self.odom_init = True

		# get my heading
		q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
		[self.my_heading, garbage, trash] = quaternions_to_RPY( q )


	def update_loc(self, data):
		# update my position in the local frame
		[self.map_x, self.map_y] = self.world_to_map( [data[0], data[1] ] )

	def world_to_map( self, point ):
		
		px = int( round( point[1] * self.cellsPerMeter ) + self.origin_y )
		py = int( round( point[0] * self.cellsPerMeter ) + self.origin_x )

		return [px, py]

	def back_to_world( self, point ):
		px = float( point[1] - self.origin_y ) / self.cellsPerMeter
		py = float( point[0] - self.origin_x ) / self.cellsPerMeter
		return [px, py]
		

	def callback_scan( self, scan ): 
		if not self.odom_init:
			if rospy.get_time() - self.waiting_on_odom_timer > self.waiting_on_odom_interval:
				self.waiting_on_odom_timer = rospy.get_time()				
				print "mapping::waiting on odom ", self.odom_z
			return		
		
		h = self.my_heading
		m_x = self.map_x
		m_y = self.map_y

		empty = np.zeros( np.shape( self.occ ) )

		# if end point is between min and max then assume obstacle
		scan.angle_min = -0.959931
		scan.angle_max = 0.959931
		scan.angle_increment = 1.91986 / float( len(scan.ranges) )
		angle = h + scan.angle_min
		self.min_scan_dist = float("inf")
		self.max_scan_dist = -float("inf")
		for i, r in enumerate(scan.ranges):

			if math.isnan(r) or i % 4 != 0 or math.isinf(r):
				angle += scan.angle_increment
				continue

			# get point scan ends
			ppx = int( round( float(m_x) - r * self.cellsPerMeter * math.sin( angle ) ) )
			ppy = int( round( float(m_y) + r * self.cellsPerMeter * math.cos( angle ) ) ) 
			angle += scan.angle_increment

			if r > 3.0 and r < 20.0:
				
				# scan hit an obstacle, probably
				self.occ[ppx][ppy] = self.bayes_update(self.occ[ppx][ppy], 0.54)
				if self.occ[ppx][ppy] > 0.65:
					#self.add_obstacle_to_map([ppx, ppy])
					a = 0
				if r < self.min_scan_dist:
					self.min_scan_dist = r
				if r > self.max_scan_dist:
					self.max_scan_dist = r

				#ppx = int( round( float(m_x) - (r * self.cellsPerMeter - 1) * math.sin( angle ) ) )
				#ppy = int( round( float(m_y) + (r * self.cellsPerMeter - 1) * math.cos( angle ) ) ) 
			
			# line iterator from min to end - 1 with free else
			pixels = self.createLineIterator( np.asarray([m_x, m_y]), np.asarray([ppx, ppy]), self.occ )
			for pixel in pixels:
				[ppx, ppy] = pixel
				ppx = int(ppx)
				ppy = int(ppy)				
				self.occ[ppx][ppy] = self.bayes_update(self.occ[ppx][ppy], 0.485)#0.4935)
				if self.occ[ppx][ppy] < 0.4:
					self.clear_obstacle_on_map([ppx, ppy])
		
			#print "min / max: ", self.min_scan_dist , ", ", self.max_scan_dist

		if rospy.get_time() - self.display_map_timer > self.display_map_interval:
			self.display_map_timer = rospy.get_time()				
		#	self.display_map()

	def add_obstacle_to_map( self, point ):
		l = int( math.ceil( self.obstacle_radius * self.cellsPerMeter ) )
		cv2.circle( self.my_map, (point[0], point[1]), l, self.obstacle, -1)

	def clear_obstacle_on_map( self, point ):
		cv2.circle( self.my_map, (point[0], point[1]), 1, self.free, -1)
		
	def bayes_update( self, prior, obsVal ):
		post = obsVal*prior/( obsVal*prior + (1-obsVal)*(1-prior) )
		return post


	def estimate_travel_costs( self, s, goals):
		start = self.world_to_map( [s.x, s.y] )
		
		costs = []
		for g in goals:
			goal = self.world_to_map( [g.x, g.y] )
			[path, length] = self.aStar( start, goal )		
			costs.append( float(length) / self.cellsPerMeter )

		return costs

	def get_path( self, s, g ):
		start = self.world_to_map( [s.x, s.y] )
		goal = self.world_to_map( [g.x, g.y] )

		[path, length] = self.aStar( start, goal )		

		if path == -1:
			return []

		self.display_path_over_map( path )		

		locs = []
		for p in path:
			l = self.back_to_world( p )
			l.append( 10.0 )
			l.append ( 0.0 )
			locs.append( Loc( l ) )
		return locs

	def aStar( self, s, g ):
		self.width = np.size( self.my_map, 0)
		self.height = np.size( self.my_map, 1)

		epsilon = 2.0
		start = State(s)
		goal = State(g)

		start.g = 0
		start.f = epsilon * self.heuristic( start, goal )
		o_set = []
		c_set = []
		o_set.append( start )
		
		while len( o_set ) > 0:
			c = self.getMindex( o_set )
			current = o_set[c]
			o_set.remove( current )
			c_set.append(current )
			#print "current: " , current.x, ", ", current.y , ", ", goal.x, ", ", goal.y , ", ", self.heuristic( current, goal )
 			if self.heuristic( current, goal ) < 1.0:
				#print "in loop"
				path = []
				length = 0
				path.append( [current.x, current.y] )
				while self.heuristic( current, start ) > 1.0:
					prev = current.came_from
					path.append( [prev.x, prev.y] )
					length += self.heuristic( prev, current )
					current = prev
					
				path.reverse()
				if length > 0:
					path.pop(0)

				return [path, length]

			nbrs = self.getNbrs( current )
			for nbr in nbrs:
				if self.my_map[nbr.y][nbr.x] != self.obstacle and not self.inList( c_set, nbr):
					if not self.inList(o_set, nbr):
						nbr.g = current.g + self.heuristic( current, nbr )
						nbr.f = nbr.g +  epsilon * self.heuristic( nbr, goal )
						nbr.came_from = current
						o_set.append( nbr )
					elif nbr.g > current.g + self.heuristic( nbr, current ):
						nbr.g = current.g + self.heuristic( current, nbr )
						nbr.f =  nbr.g + self.heuristic( nbr, goal )
						nbr.came_from = current

		return [-1, float("inf")]

	def getNbrs( self, c ):
		nbrs = []
		
		nx = [-1,-1,-1,0,0,1,1,1]
		ny = [-1,0,1,-1,1,-1,0,1]
	
		for x,y in zip(nx, ny):
			n = State([c.x + x, c.y+y])
			if n.x > -1 and n.x < self.width and n.y > -1 and n.y < self.height and self.my_map[n.y][n.x] != self.obstacle:
				nbrs.append( n )
		return nbrs


	def inList( self, list, item ):
		for s in list:
			if s.x == item.x and s.y == item.y:
				return True

		return False

	def getMindex( self, set ):
		min = float("inf")
		mindex = -1
		for i,s in enumerate( set ):
			if s.f < min:
				min = s.f
				mindex = i

		return mindex

	def heuristic( self, a, b ):
		return math.sqrt( pow(float(a.x-b.x),2) + pow(float(a.y-b.y),2) )
		
	def display_map( self ):

		display = np.empty_like( self.my_map )
		np.copyto( display, self.my_map )

		cv2.circle( display, (int(self.map_x), int(self.map_y) ), int(2.0 * self.cellsPerMeter), 200, -1)
		

		cv2.namedWindow("Map", cv2.WINDOW_NORMAL)
		cv2.imshow("Map", display)
		cv2.waitKey( 10 )
    
	def display_path_over_map( self, path ):

		display = np.empty_like( self.my_map )
		np.copyto( display, self.my_map )
		
		for p in path:
			cv2.circle( display, (int(round(p[0])), int(round(p[1]))), 1, 200, -1)

		cv2.circle( display, (int(path[0][0]), int(path[0][1])), 3, 200, -1)
		cv2.circle( display, (int(path[-1][0]), int(path[-1][1])), 3, 50, -1)
				
		cv2.namedWindow("Path over map", cv2.WINDOW_NORMAL)
		cv2.imshow("Path over map", display)
		cv2.waitKey( 10 )

	def createLineIterator(self, P1, P2, img):
		"""
		Produces and array that consists of the coordinates and intensities of each pixel in a line between two points

		Parameters:
			-P1: a numpy array that consists of the coordinate of the first point (x,y)
			-P2: a numpy array that consists of the coordinate of the second point (x,y)
			-img: the image being processed

		Returns:
			-it: a numpy array that consists of the coordinates and intensities of each pixel in the radii (shape: [numPixels, 3], row = [x,y,intensity])     
		"""
		#define local variables for readability
		imageH = img.shape[0]
		imageW = img.shape[1]
		P1X = P1[0]
		P1Y = P1[1]
		P2X = P2[0]
		P2Y = P2[1]

		#difference and absolute difference between points
		#used to calculate slope and relative location between points
		dX = P2X - P1X
		dY = P2Y - P1Y
		dXa = np.abs(dX)
		dYa = np.abs(dY)

		#predefine numpy array for output based on distance between points
		itbuffer = np.empty(shape=(np.maximum(dYa,dXa),2),dtype=np.float32)
		itbuffer.fill(np.nan)

		#Obtain coordinates along the line using a form of Bresenham's algorithm
		negY = P1Y > P2Y
		negX = P1X > P2X
		if P1X == P2X: #vertical line segment
			itbuffer[:,0] = P1X
			if negY:
				itbuffer[:,1] = np.arange(P1Y - 1,P1Y - dYa - 1,-1)
			else:
				itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)              
		elif P1Y == P2Y: #horizontal line segment
			itbuffer[:,1] = P1Y
			if negX:
				itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
			else:
				itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
		else: #diagonal line segment
			steepSlope = dYa > dXa
			if steepSlope:
				slope = dX.astype(np.float32)/dY.astype(np.float32)
				if negY:
					itbuffer[:,1] = np.arange(P1Y-1,P1Y-dYa-1,-1)
				else:
					itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
				itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(np.int) + P1X
			else:
				slope = dY.astype(np.float32)/dX.astype(np.float32)
				if negX:
					itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
				else:
					itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
				itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(np.int) + P1Y

		#Remove points outside of image
		colX = itbuffer[:,0]
		colY = itbuffer[:,1]
		itbuffer = itbuffer[(colX >= 0) & (colY >=0) & (colX<imageW) & (colY<imageH)]
		
		return itbuffer




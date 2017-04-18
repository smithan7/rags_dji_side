import numpy as np
import cv2
import math
import time
import rospy
from State import State
from navigation import Loc

class Planner(object):
	"""description of class"""

	my_map = np.array
	cellsPerMeter = 1.0

	def __init__( self, cpm ):
		self.cellsPerMeter = cpm

	def estimate_travel_costs( self, s, goals, my_map, origin_x, origin_y ):
		self.my_map = my_map
		self.origin_x = origin_x
		self.origin_y = origin_y
		start = self.world_to_map( [s.x, s.y] )
		
		costs = []
		for g in goals:
			goal = self.world_to_map( [g.x, g.y] )
			[path, length] = self.aStar( start, goal )		
			costs.append( float(length) / self.cellsPerMeter )

		return costs

	def world_to_map( self, point ):
		#print "point: " , point
		#print "cells2meters: " , self.cellsPerMeter
		#print "origin: " , self.origin_x, ", " , self.origin_y

		px = int( round( point[1] * self.cellsPerMeter ) + self.origin_y )
		py = int( round( point[0] * self.cellsPerMeter ) + self.origin_x )

		#print "map_shape: ", np.shape( self.my_map )

		#print "px/y: ", px, ", ", py

		#print "back?: ", self.back_to_world( [px, py] )

		return [px, py]

	def back_to_world( self, point ):
		px = float( point[1] - self.origin_y ) / self.cellsPerMeter
		py = float( point[0] - self.origin_x ) / self.cellsPerMeter
		return [px, py]

	def get_path( self, s, g, mp ):

		self.my_map = mp
		start = self.world_to_map( [s.x, s.y] )
		goal = self.world_to_map( [g.x, g.y] )

		[path, length] = self.aStar( start, goal )		

		if path == -1:
			return []

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
				if self.my_map[nbr.x, nbr.y] <= 200 and not self.inList( c_set, nbr):
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
			if n.x > -1 and n.x < self.width and n.y > -1 and n.y < self.height and self.my_map[n.x,n.y] <= 200:
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
		
	def displayMap( self ):

		cv2.namedWindow("fm2 Map", cv2.WINDOW_NORMAL)
		cv2.imshow("fm2 Map", self.my_map)
		cv2.waitKey( 10 )
    
	def displayPath_over_map( self, path ):

		display = np.zeros( (len(self.speed), len(self.speed[0]), 3), np.uint8)
		for i in range(0, np.size( self.my_map, 0) ):
			for j in range(0, np.size( self.my_map, 1) ):
				if self.my_map[i][j] == 255:
					display[i][j][0] = 0
					display[i][j][1] = 0
					display[i][j][2] = 0
				else:
					display[i][j][0] = 255
					display[i][j][1] = 255
					display[i][j][2] = 255

		for p in path:
			cv2.circle( display, (round(p[1]), round(p[0])), 2, (0,0,255), -1)

		cv2.circle( display, (path[0,0], path[0,1]), 5, (0,0,255), -1)
		cv2.circle( display, (path[-1,0], path[-1,1]), 5, (0,0,255), -1)

		cv2.namedWindow("fm2 path over map", cv2.WINDOW_NORMAL)
		cv2.imshow("fm2 path over map", display)
		cv2.waitKey(10)


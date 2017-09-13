#!/usr/bin/env python


import math

class Global_Loc(object):
	def __init__( self, lon, lat):
		self.longitude = lon
		self.latitude = lat
		
class Local_Loc(object):
	def __init__(self, x,y,z,w):
		self.local_x = x
		self.local_y = y
		self.altitude = z
		self.heading = w

def euclid2d(a,b):
	d = math.sqrt((a.local_x-b.local_x)**2 + (a.local_y-b.local_y)**2)
	return d


def euclid3d(a,b):
	d = math.sqrt((a.local_x-b.local_x)**2 + (a.local_y-b.local_y)**2 + (a.altitude-b.altitude)**2)
	return d

def GPS_to_local( a, origin): # from lat/lon to quads local map in meters
	#print("origin: ", origin.longitude, ", ", origin.latitude)
	d = distance_from_a_to_b( origin, a )
	b = heading_from_a_to_b( origin, a )
	x = d*math.sin(b)
	y = d*math.cos(b)
	return [x, y]

def local_to_GPS(x, y, origin_lati, origin_longti): # from quads local map in meters to lat/lon
	C_EARTH = 6378136.6
	dlati = x / C_EARTH
	lati = dlati + origin_lati
	dlongti = y / ( C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0) )
	longti = dlongti + origin_longti

	return [lati, longti]

def quaternions_to_RPY( q ):
	roll  = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]) , 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]) )
	pitch = math.asin(2.0 * (q[2] * q[0] - q[3] * q[1]) )
	yaw   = math.atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1]) )

	return [roll, pitch, yaw]

def heading_from_a_to_b( a, b ):
	lat1 = math.radians(a.latitude)
	lat2 = math.radians(b.latitude)

	diffLong = math.radians(b.longitude - a.longitude)

	x = math.sin(diffLong) * math.cos(lat2)
	y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
		    * math.cos(lat2) * math.cos(diffLong))

	heading = math.atan2(x, y)

	return heading

def distance_from_a_to_b( a, b ):
	R = 6378136.6 # radius of the earth in meters

	lat1 = math.radians(a.latitude)
	lon1 = math.radians(a.longitude)
	lat2 = math.radians(b.latitude)
	lon2 = math.radians(b.longitude)

	dlon = lon2 - lon1
	dlat = lat2 - lat1

	a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

	distance = R * c # in meters
	return distance



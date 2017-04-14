#!/usr/bin/env python


import math

def GPS_to_local( lati, longti, origin_lati, origin_longti): # from lat/lon to quads local map in meters
	C_EARTH = 6378137.0
	dlati = lati-origin_lati
	dlongti= longti-origin_longti
	x = dlati * C_EARTH
	y = dlongti * C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0)

	return [x, y]

def local_to_GPS(x, y, origin_lati, origin_longti): # from quads local map in meters to lat/lon
	C_EARTH = 6378137.0
	dlati = x / C_EARTH
	lati = d_lati + origin_lati
	dlongti = y / ( C_EARTH * math.cos(lati / 2.0 + origin_lati / 2.0) )
	longti = dlongti + origin_longti

	return [lati, longti]

def local_to_map( x, y, offset_x, offset_y, resolution ): # from quads local map in meters to gmapping's map
	ix = (x-offset_x) / resolution
	iy = (y-offset_y) / resolution
	return [ix, iy]

def map_to_local( ix, iy, offset_x, offset_y, resolution ): # from gmapping map to quads map in meters
	x = ix * resolution + offset_x
	y = iy * resolution + offset_y
	return [x, y]

def quaternions_to_RPY( q ):
	roll  = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]) , 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]) )
	pitch = math.asin(2.0 * (q[2] * q[0] - q[3] * q[1]) )
	yaw   = math.atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1]) )

	return [roll, pitch, yaw]

def heading_from_a_to_b( a_lat, a_lon, b_lat, b_lon ):
	lat1 = math.radians(a_lat)
	lat2 = math.radians(b_lat)

	diffLong = math.radians(b_lon - a_lon)

	x = math.sin(diffLong) * math.cos(lat2)
	y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
		    * math.cos(lat2) * math.cos(diffLong))

	initial_bearing = math.atan2(x, y)
	initial_bearing = math.degrees(initial_bearing)
	compass_bearing = (initial_bearing + 360) % 360

	return compass_bearing

def distance_from_a_to_b( a_lat, a_lon, b_lat, b_lon ):
	R = 6378137.0 # radius of the earth in meters

	lat1 = math.radians(a_lat)
	lon1 = math.radians(a_lon)
	lat2 = math.radians(b_lat)
	lon2 = math.radians(b_lon)

	dlon = lon2 - lon1
	dlat = lat2 - lat1

	a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

	distance = R * c # in meters
	return distance

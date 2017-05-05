#!/usr/bin/env python


import math

def GPS_to_local( lati, longti, origin_lati, origin_longti): # from lat/lon to quads local map in meters
	d = distance_from_a_to_b( origin_lati, origin_longti, lati, longti )
	b = heading_from_a_to_b( origin_lati, origin_longti, lati, longti )

	x = -d*math.sin(b)
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

def heading_from_a_to_b( a_lat, a_lon, b_lat, b_lon ):
	lat1 = math.radians(a_lat)
	lat2 = math.radians(b_lat)

	diffLong = math.radians(b_lon - a_lon)

	x = math.sin(diffLong) * math.cos(lat2)
	y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
		    * math.cos(lat2) * math.cos(diffLong))

	initial_bearing = math.atan2(x, y)

	return initial_bearing

def distance_from_a_to_b( a_lat, a_lon, b_lat, b_lon ):
	R = 6378136.6 # radius of the earth in meters

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

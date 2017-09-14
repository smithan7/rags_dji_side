#!/usr/bin/env python
from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from dji_sdk.msg import GlobalPosition
from dji_sdk.msg import TransparentTransmissionData
from dji_sdk.msg import RCChannels


from quad import quad


import time
import sys
import math

if __name__ == "__main__":
	drone = DJIDrone()
	env_num = rospy.get_param('test_environment_number')

	nw_corner = []
	se_corner = []

	# practice field
	if env_num == 0:
		nw_corner = [44.565683, -123.272974]
		se_corner = [44.564965, -123.270456]


	# hardware test environment
	if env_num == 3:	
		nw_corner = [44.539847, -123.251004]#rospy.get_param( '~nw_corner' )
		se_corner = [44.538552, -123.247446]#rospy.get_param( '~se_corner' )


	# new hardware4
	if env_num == 4:	
		nw_corner = [44.5394569, -123.250866]
		se_corner = [44.538162, -123.247776]

	# new hardware4
	if env_num == 5:	
		nw_corner = [44.539580, -123.250876]
		se_corner = [44.538725, -123.247601]

	#rospy.init_node("DJI_Bridge")
	#Q = quad( nw_corner, se_corner )	
	print("DJI_Bridge::Initializing")
	
	Q = quad( drone, nw_corner, se_corner )
	
	rospy.spin()

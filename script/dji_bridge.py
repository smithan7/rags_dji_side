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
	nw_corner = [44.539847, -123.251004]#rospy.get_param( '~nw_corner' )
	se_corner = [44.538552, -123.247446]#rospy.get_param( '~se_corner' )

	rospy.init_node("DJI_Bridge")
	#drone = DJIDrone()
	#Q = quad( drone, nw_corner, se_corner )
	Q = quad( nw_corner, se_corner )	
	rospy.spin()

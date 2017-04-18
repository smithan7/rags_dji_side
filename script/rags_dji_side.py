#!/usr/bin/env python

from utils import *
from rags_quad import rags_quad

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
from dji_sdk.msg import GlobalPosition
from dji_sdk.msg import TransparentTransmissionData
from dji_sdk.msg import RCChannels

import time
import sys
import math

if __name__ == "__main__":
	#rospy.init_node('rags_dji_side', anonymous=False)
	drone = DJIDrone()
	use_rags = rospy.get_param( '~use_rags' )	
	print("use rags: ", use_rags)	
	rags_Q = rags_quad( drone, use_rags )
	
	rospy.spin()

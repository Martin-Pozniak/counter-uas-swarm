#!/usr/bin/env python

'''
# Author:           Martin Pozniak
# Creation Date:    2/23/19
# Last Edit Date:   3/31/19
# Description:      Simulates a target UAV for the system to detect and mitigate.
'''

# ===================Imports==============================================
# ========================================================================

import rospy
from drone_brain import *

# ===================Functions==============================================
# ==========================================================================

def print_stats(stat):
    stats = vehicle.get_stats()
    if(stat is None):           
        print stats
    else:
        print stats[stat]

# ===================ONBOARD CONTROL======================================
# ========================================================================

vehicle = DroneBrain() 
rate = rospy.Rate(100)

rospy.loginfo("Setting Mode To Offboard")
vehicle.set_mode("OFFBOARD")
rospy.loginfo("Mode Successfully Set To Offboard")

uav_id = vehicle.get_stats()['id']

desired_alt = 1

# Once dynamic positioning is implemented we can remove the below ID specific instructions. 
# Position will be determined relative to the UAV elected as master.

if uav_id == 6: 
    rospy.loginfo("==UAV TAKEOFF==")
    vehicle.takeoff(desired_alt)  # Takeoff function will ARM the UAV
    while not rospy.is_shutdown():

        vehicle.move_to(0, 0, 2)
        # vehicle.move_to(0, -3, 2)

        rate.sleep()

    vehicle.set_mode("RTL")    

rospy.loginfo("==SCRIPT FINISHED==")
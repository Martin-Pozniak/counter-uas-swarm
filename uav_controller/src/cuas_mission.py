#!/usr/bin/env python

'''
# Author:           Martin Pozniak
# Creation Date:    2/23/19
# Last Edit Date:   4/15/19
# Description:      cuas_mission.py instantiates a new drone by calling drone_brain.py.
#                   This file runs all on-board control for each drone to complete a mission.
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

if uav_id == 1: 
    rospy.loginfo("==UAV TAKEOFF==")
    vehicle.takeoff(desired_alt)  # Takeoff function will ARM the UAV
    while not rospy.is_shutdown():

        vehicle.move_to(0, 0, 2)

        rate.sleep()
    vehicle.set_mode("RTL")  

elif uav_id == 2:
    vehicle.takeoff(desired_alt)
    count = 0
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()
        
        # vehicle.follow(1, -2, 2, 0, None)
        vehicle.move_to(-4,-2,2)

        rate.sleep()
    vehicle.set_mode("RTL")

elif uav_id == 3:
    vehicle.takeoff(desired_alt)
    count = 0
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()
        
        vehicle.move_to(-6,2,2)

        rate.sleep()
    vehicle.set_mode("RTL")

elif uav_id == 4:
    vehicle.takeoff(desired_alt)
   
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()

        # vehicle.follow(1, -4, 0, 0, None)
        vehicle.move_to(-6,-2,2)

        rate.sleep()
    vehicle.set_mode("RTL")

elif uav_id == 5: 
    vehicle.takeoff(desired_alt)
    
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()

        # vehicle.follow(1, -7, 1, 0, None)
        vehicle.move_to(-8,2,2)

        rate.sleep()
    vehicle.set_mode("RTL")


rospy.loginfo("==SCRIPT FINISHED==")

# Register Each UAV To the network
# Begin Forming a Formation
# Execute Search
# Once Target is Found, Surround It Relay Message To Ground Station
#!/usr/bin/env python

'''
# Author:           Martin Pozniak
# Creation Date:    2/23/19
# Last Edit Date:   2/23/19
# Description:      controller.py instantiates a new drone by calling drone_brain.py.
#                   This file runs all on-board control for each drone to complete a mission.
'''

# ===================Imports==============================================
# ========================================================================

import rospy
from drone_brain import *

# ===================Functions==============================================
# ========================================================================
def print_stats(stat):
    stats = vehicle.get_stats()
    if(stat is None):           
        print stats
    else:
        print stats[stat]

# ===================ONBOARD CONTROL======================================
# ========================================================================

vehicle = DroneBrain()  # Initialize this drone and give it access to all vehicle controls from drone_brain.py
rate = rospy.Rate(20)

rospy.loginfo("==Setting Mode To Offboaard==")
vehicle.set_mode("OFFBOARD")
rospy.loginfo("==Mode set to offboard==")

uav_id = vehicle.get_stats()['id']

desired_alt = 1

if uav_id == 1: 
    rospy.loginfo("==UAV TAKEOFF==")
    vehicle.takeoff(desired_alt)  # Takeoff function will ARM the UAV
    while not rospy.is_shutdown():        

        rospy.loginfo("==Moving To Home==")

        vehicle.move_to(0, 0, 2)

        rospy.loginfo("==Moving To Position 1==")
        
        vehicle.move_to(-10, -10, 3)
            
        rospy.loginfo("==Moving To Position 2==")

        vehicle.move_to(10, -10, 2)

        rospy.loginfo("==Moving To Position 3==")

        vehicle.move_to(10, 10, 3)

        rospy.loginfo("==Moving To Position 4==")

        vehicle.move_to(-10, 10, 1)

        rospy.loginfo("==Moving To Position 5==")
        
        vehicle.move_to(-10, -10, 3)

        rate.sleep()
        
elif uav_id == 2:
    vehicle.takeoff(desired_alt)
    # time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()

        vehicle.follow(1, -1, 1, 0, None)

        rate.sleep()

elif uav_id == 3:
    vehicle.takeoff(desired_alt)
    time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()

        vehicle.follow(1, -5, -1, 0, None)

        rate.sleep()

elif uav_id == 4:
    vehicle.takeoff(desired_alt)
    time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()

        vehicle.follow(1, -6, 0, 0, None)
        rate.sleep()

elif uav_id == 5: 
    vehicle.takeoff(desired_alt)
    time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():

        stats = vehicle.get_stats()

        vehicle.follow(1, -7, 1, 0, None)

        rate.sleep()

rospy.loginfo("==SCRIPT FINISHED==")
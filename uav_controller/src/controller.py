#!/usr/bin/env python

'''
# Author:           Martin Pozniak
# Creation Date:    2/23/19
# Last Edit Date:   2/23/19
# Description:      controller.py instantiates a new drone by calling drone_brain.py.
#                   This file runs all on-board control for each drone to complete a mission.


This commit includes the packages required to run the first stage of the dynamic ad-hoc UAV swarm. 

The files allow for any number of simulated UAVs running PX4 to be loaded into a gazebo world. The UAVs will position themselves into formation and follow the acting master in a square pattern.

The swarm nodes each communicate with one another. Each node in the network knows the position and velocity of every other node in the network. This allows for inter-swarm collision avoidance to be implemented. (which is next)
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
rate = rospy.Rate(10)

rospy.loginfo("==Setting Mode To Offboaard==")
vehicle.set_mode("OFFBOARD")
rospy.loginfo("==Mode set to offboard==")

uav_id = vehicle.get_stats()['id']

desired_alt = 1

if uav_id == 1: 
    while not rospy.is_shutdown():        

        rospy.loginfo("==UAV TAKEOFF==")
        vehicle.set_velocity(20)
        vehicle.takeoff(desired_alt)  # Takeoff function will ARM the UAV

        rospy.loginfo("==Moving To Home==")

        vehicle.move_to(-1, -1, 2, 0.4)

        rospy.loginfo("==Moving To Position 1==")
        
        vehicle.move_to(-5, -5, 3, 0.4)
            
        rospy.loginfo("==Moving To Position 2==")

        vehicle.move_to(5, -5, 2, 0.4)

        rospy.loginfo("==Moving To Position 3==")

        vehicle.move_to(5, 5, 3, 0.4)

        rospy.loginfo("==Moving To Position 4==")

        vehicle.move_to(-5, 5, 1, 0.4)

        rate.sleep()
        
elif uav_id == 2:
    vehicle.set_velocity(50)
    vehicle.takeoff(desired_alt)
    time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():
        stats = vehicle.get_stats()
        # print "======UAV " + str(stats['id']) + "======"
        # print stats['battery_voltage']
        vehicle.follow(1, -2, 2, 0, None)
        rate.sleep()
elif uav_id == 3:
    vehicle.set_velocity(50)
    vehicle.takeoff(desired_alt)
    time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():
        stats = vehicle.get_stats()
        # print "======UAV " + str(stats['id']) + "======"
        # print stats['battery_voltage']
        vehicle.follow(1, -2, 2, 0, None)
        rate.sleep()
elif uav_id == 4:
    vehicle.set_velocity(50)
    vehicle.takeoff(desired_alt)
    time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():
        stats = vehicle.get_stats()
        # print "======UAV " + str(stats['id']) + "======"
        # print stats['battery_voltage']
        vehicle.follow(1, -4, 0, 0, None)
        rate.sleep()
elif uav_id == 5: 
    vehicle.set_velocity(50)
    vehicle.takeoff(desired_alt)
    time.sleep(4)
    # vehicle.wait_for_position_from(uav=1, alt=2) # UAVs publish to a topic when they successfully reach their target location. We can use that to wait
    while not rospy.is_shutdown():
        stats = vehicle.get_stats()
        print "======UAV " + str(stats['id']) + "======"
        # print stats['battery_voltage']
        vehicle.follow(1, -7, 1, 0, None)
        rate.sleep()

rospy.loginfo("==SCRIPT FINISHED==")
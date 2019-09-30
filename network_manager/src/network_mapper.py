#!/usr/bin/env python

'''
# Author:           Martin Pozniak
# Creation Date:    2/26/19
# Last Edit Date:   3/3/19
# Description:      Network_Mapper continuosly monitors for nodes within the network and publishes to /network_map topic 
#                   when it detects a change in the network. Each UAV runs a network mapper program to keep track of nodes within their network.
#
# Changes For This Commit:
#   - Updated file name to be more consistant with functionality. Renamed from swarm_node_count
#   - Dynamically grabs IP addresses from the host device
#   - Grabs the number of hosts and IPs of each host within the network
#   - Need to look into speed optimization
'''

# ===================Imports==============================================
# ========================================================================
import rospy
import time
import sys
from std_msgs.msg import Int32
import subprocess
import netifaces as ni

# ===================Functions ===========================================
# ========================================================================
def start_ping_sweep():
    
    cmdping = "ping -c1 " + a + "." + b + "." + c + "."

    for x in range (2,255):
        p = subprocess.Popen(cmdping+str(x), shell=True, stderr=subprocess.PIPE)

        while True:
            out = p.stderr.read(1)
            if out == '' and p.poll() != None:
                break
            if out != '':
                sys.stdout.write(out)
                sys.stdout.flush()
    

# ===================Initialization=======================================
# ========================================================================
rospy.init_node("network_mapper")
rate = rospy.Rate(2) # Publish Updates Every 2 Seconds

node_topic_pub = rospy.Publisher("/network_map", Int32)

num_nodes = Int32()

num_nodes.data = input("Enter Number of Nodes To Test: ")

# ===================Keep Node Alive======================================
# ========================================================================
while not rospy.is_shutdown():

    # Discover hosts on the network
    # Take the num of hosts
    #start_ping_sweep()
    node_topic_pub.publish(num_nodes)
    rate.sleep()
    

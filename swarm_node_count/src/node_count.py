#!/usr/bin/env python

'''
# Author:           Martin Pozniak
# Creation Date:    2/26/19
# Last Edit Date:   2/26/19
# Description:      Node_count continuosly monitors for nodes within the network and publishes to /node_count topic 
#                   when it detects a change in the network. 
'''

# ===================Imports==============================================
# ========================================================================
import rospy
import time
import sys
from std_msgs.msg import Int32

# ===================Functions ===========================================
# ========================================================================
def join_request_handler(request_data):
    rospy.loginfo("New UAV with ID: " + str(request_data) + " has joined the network")
    count += 1
    node_topic_pub.Publish(count)

# ===================Initialization=======================================
# ========================================================================
rospy.init_node("node_counter")
rate = rospy.Rate(0.5)

node_topic_pub = rospy.Publisher("/swarm_node_count", Int32)
node_join_request_sub = ("/join _swarm", Int32, join_request_handler)

# ===================Keep Node Alive======================================
# ========================================================================
num_nodes = Int32()
num_nodes.data = input("Enter Number of Nodes To Test: ")
while not rospy.is_shutdown():
    node_topic_pub.publish(num_nodes)
    rate.sleep()
    

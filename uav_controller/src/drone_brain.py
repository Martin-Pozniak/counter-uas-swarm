#!/usr/bin/env/ python

'''
# Author:           Martin Pozniak
# Creation Date:    2/23/19
# Last Edit Date:   2/26/19
# Description:      drone_brain.py initializes a new ROS node and contains all functions to control the UAV.
#                   It runs onboard the UAV.
'''

# ===================Imports==============================================
# ========================================================================
import rospy
import time
import sys
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, GlobalPositionTarget
from sensor_msgs.msg import BatteryState, NavSatFix

# ===============DroneBrain Class=========================================
# ========================================================================


class DroneBrain:
    # ===============Constructor Function=====================================
    # ========================================================================
    def __init__(self):

        # Initialize the UAV
        # Declare Variables
        self.id = input("Drone ID #: ")
        self.uav_id = "uav" + str(self.id)

        rospy.init_node("controller" + str(self.id), anonymous=True)

        self.rate = rospy.Rate(10)  

        # Create required publsihers for control
        self.target_pos_pub = rospy.Publisher(self.uav_id + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.target_vel_pub = rospy.Publisher(self.uav_id + "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        

        # Create required subcribers for information and control
        self.state_sub = rospy.Subscriber(self.uav_id + "/mavros/state", State, self.state_changed_callback)
        self.actual_pos_sub = rospy.Subscriber(self.uav_id + "/mavros/local_position/pose", PoseStamped, self.position_callback)
        self.actual_vel_sub = rospy.Subscriber(self.uav_id + "/mavros/local_position/velocity", TwistStamped, self.velocity_callback)
        self.battery_voltage_sub = rospy.Subscriber(self.uav_id + "/mavros/battery", BatteryState, self.battery_voltage_callback)
        self.network_state_sub = rospy.Subscriber("/swarm_node_count", Int32, self.network_state_changed_callback)
        self.neighbor_pos_sub = {}
        self.neighbor_vel_sub = {}  # Initialized dynamically based on number of nodes in net

        # Create Objects to pair with above pubs and subs    
        self.actual_velocity_object = TwistStamped() 
        self.battery_voltage_object = BatteryState()   
        self.desired_pos_object = PoseStamped()
        self.desired_velocity_object = Twist()
        self.actual_pos_object = PoseStamped()
        self.neighbor_position_objects = {}
        self.num_nodes_in_swarm = Int32()
        self.mode_object = SetMode()
        self.state = State()       

    # ====================Callback Functions==================================
    # ========================================================================
    def neighbor_position_changed_callback(self, position_object, index):
        self.neighbor_position_objects[index] = position_object

    def neighbor_velocity_changed_callback(self, velocity_object, index):
        pass

    def network_state_changed_callback(self, new_num_nodes):
        self.num_nodes_in_swarm = new_num_nodes
        rospy.loginfo("THERE ARE " + str(self.num_nodes_in_swarm.data) + " NODES IN THE SWARM")
        for i in range(1, self.num_nodes_in_swarm.data + 1):
                # Create subs for each node thats in the swarm (except outself) so we can monitor each of their positions           
                if (i != self.id):
                    if not "uav"+str(i) in self.neighbor_position_objects.keys():
                        rospy.loginfo("Registering UAV " + str(i) + " in my network FROM CALLBACK")
                        self.neighbor_position_objects["uav"+str(i)] = PoseStamped()
                        self.neighbor_velocity_objects["uav"+str(i)] = TwistStamped()

                    if not i in self.neighbor_position_objects.keys():
                        self.neighbor_pos_sub[i] = rospy.Subscriber("uav" + str(i) + "/mavros/local_position/pose", PoseStamped, self.neighbor_position_changed_callback, i )
                        self.neighbor_vel_sub[i] = rospy.Subscriber("uav" + str(i) + "/mavros/local_position/velocity", TwistStamped, self.neighbor_velocity_changed_callback, i)
                    #TRIPLE CHECK THERES NO SCOPING ISSUES CREATED HERE

    def state_changed_callback(self, state_object):
        self.state = state_object
    
    def position_callback(self, position_object):
        # rospy.loginfo("POSITION CALLBACK" + str(position_object.pose.position.z))
        self.actual_pos_object = position_object

    def velocity_callback(self, velocity_object):
        # rospy.loginfo("Velocity CALLBACK", velocity_object)
        self.actual_velocity_object = velocity_object
    
    def battery_voltage_callback(self, battery_voltage):
        self.battery_voltage_object.voltage = battery_voltage

    # ====================Arming Function=====================================
    # ========================================================================
    def arm(self):
        cmd = CommandBool()
        cmd.value = True
        self.arming_service_client = rospy.ServiceProxy("uav" + str(self.id) + "/mavros/cmd/arming", CommandBool)
        while not self.arming_service_client(cmd.value).success is True:
            rospy.loginfo("UAV " + str(self.id) + " waiting to arm")

            for i in range(100):
                self.target_pos_pub.publish(self.desired_pos_object)
                self.rate.sleep()

            self.rate.sleep()

        rospy.loginfo("UAV " + str(self.id) + " ARMED")

    # ====================in_position Function================================
    # ========================================================================
    def in_position(self):
        self.rate.sleep()
        if (abs(self.desired_pos_object.pose.position.x - self.actual_pos_object.pose.position.x) < 0.1 and  
                abs(self.desired_pos_object.pose.position.y - self.actual_pos_object.pose.position.y) < 0.1 and
                abs(self.desired_pos_object.pose.position.z - self.actual_pos_object.pose.position.z) < 0.1):
            # print str(self.desired_pos_object.pose.position.z) + " / " + str(self.actual_pos_object.pose.position.z)
            return True
        else: 
            return False
        
    # ====================Set Velocity Function===============================
    # ========================================================================
    def set_velocity(self, velocity):
        self.desired_velocity_object.linear.x = velocity
        self.desired_velocity_object.linear.y = velocity
        self.desired_velocity_object.linear.z = velocity
        self.desired_velocity_object.angular.x = velocity
        self.desired_velocity_object.angular.y = velocity
        self.desired_velocity_object.angular.z = velocity

        for i in range(100):
            self.target_vel_pub.publish(self.desired_velocity_object)
            self.rate.sleep()

    # ====================Print Stats Function===============================
    # ========================================================================
    def get_stats(self):
        stats = {
            'id': self.id,
            'position': self.actual_pos_object,
            'velocity': self.actual_velocity_object.twist,
            'battery_voltage': self.battery_voltage_object.voltage.voltage
        }
        return stats

    # ====================Set Velocity Function===============================
    # ========================================================================
    def set_mode(self, mode_string):
        self.set_mode_srv_client = rospy.ServiceProxy(self.uav_id + "/mavros/set_mode", SetMode)

        self.mode_object.custom_mode = mode_string

        for i in range(100):
            self.target_pos_pub.publish(self.desired_pos_object)
            self.rate.sleep()
            
        rospy.loginfo("TRYING TO SET MODE " + mode_string)
        while not self.state.mode == mode_string:
            self.set_mode_srv_client(0, self.mode_object.custom_mode)
            rospy.loginfo("MODE IS " + self.state.mode + " SHOULD BE " + mode_string)

            if self.state.armed is not True:
                rospy.loginfo("UAV was disarmed. Attempting - Rearm.")
                self.arm()

            for i in range(100):
                self.target_pos_pub.publish(self.desired_pos_object)
                self.rate.sleep()

        rospy.loginfo("MODE SUCCESSFULLY CHANGED")

    # ================Dis-Arming Function=====================================
    # ========================================================================
    def disarm(self):
        pass

    # ====================Takeoff Function====================================
    # ========================================================================
    def takeoff(self, alt):
        if self.state.armed is not True:
                rospy.loginfo("UAV was disarmed. Attempting - Rearm.")
                self.arm()
        
        self.desired_pos_object.pose.position.z = alt

        print str(self.desired_pos_object.pose.position.z) + " / " + str(self.actual_pos_object.pose.position.z)

        while not self.in_position():
            self.target_pos_pub.publish(self.desired_pos_object)
    
    # ======================Obrit Function====================================
    # ========================================================================
    def orbit(self, radius):
        pass

    # ======================Wait For Position From Function===================
    # ========================================================================
    def wait_for_position_from(self, uav=None, alt=None):

        uav_name = "uav" + str(uav)
        self.uav_follow_sub = rospy.Subscriber(uav_name + "/mavros/local_position/pose", PoseStamped, self.wait_for_position_from_callback)

    def wait_for_position_from_callback(self, target_position_object):
        if self.id != 1:
            self.desired_pos_object = target_position_object
            
    # ====================Move To Function===================================
    # =======================================================================
    def move_to(self, x_loc, y_loc, z_loc, velocity=None):

        if(velocity is not None):
            self.set_velocity(velocity)
        
        self.desired_pos_object.pose.position.x = x_loc
        self.desired_pos_object.pose.position.y = y_loc
        self.desired_pos_object.pose.position.z = z_loc
        while not self.in_position():
            self.target_pos_pub.publish(self.desired_pos_object)

    # ====================Follow Function====================================
    # =======================================================================
    def follow(self, uav, x_offset, y_offset, z_offset, formation=None):

        uav_name = "uav" + str(uav)
        self.uav_follow_sub = rospy.Subscriber(uav_name + "/mavros/local_position/pose", PoseStamped, self.follow_callback)

        if self.state.armed is not True:
                rospy.loginfo("UAV was disarmed. Attempting - Rearm.")
                self.arm()

        x = self.desired_pos_object.pose.position.x
        y = self.desired_pos_object.pose.position.y
        z = self.desired_pos_object.pose.position.z

        x_new = x + x_offset
        y_new = y + y_offset
        z_new = z + z_offset

        print "x: " + str(round(x, 3)) + "/" + str(round(x_new, 3)) + " y: " + str(round(y, 3)) + "/" + str(round(y_new, 3)) + " Alt: " + str(round(z, 3)) + "/" + str(round(z_new,3))

        self.desired_pos_object.pose.position.x += x_offset
        self.desired_pos_object.pose.position.y += y_offset
        self.desired_pos_object.pose.position.z += z_offset

        self.target_pos_pub.publish(self.desired_pos_object)

    def follow_callback(self, target_position_object):
        if self.id != 1:
            self.desired_pos_object = target_position_object

# ========================================================================

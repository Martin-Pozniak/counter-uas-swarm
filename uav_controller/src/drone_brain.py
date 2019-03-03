#!/usr/bin/env/ python

'''
# Author:           Martin Pozniak
# Creation Date:    2/23/19
# Last Edit Date:   3/3/19
# Description:      drone_brain.py initializes a new ROS node and contains all functions to control the UAV.
#                   It runs onboard the UAV.
#
# Changes For This Commit:
#   - *Potentially* Solved issue where UAV stops getting updated other position information when attempting to avoid collisions 
#   - Added flag to prevent publishing any movement other than collision avoidance when collision avoidance is necessary.
#   - Removed Collision Avoidance Algorithm. Needs Serious Work
#   - Basic Cleaning
#   - Changed keys for uav neighbor position objects to 1,2,3,4 instead of uav1,uav2,uav3 for simpler code. Keys are strings since there was weird behavior with ints.
#   - Added functionality to changed failsafe modes
#   - Added Ability to get and set px4 parameters
#
# Notes: MultiMaster FKIE
#   Determine how messages will be communicated between hardware devices.
#       - How are topics published and subcribed to between machines.
#   To connect to a physical PX4, roslaunch mavros and use /dev/ttyACM0 or whatever the port is.
'''

# ===================Imports==============================================
# ========================================================================
import rospy
import time
import sys
import math
from std_msgs.msg import Int32, Int64
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode,  ParamGet, ParamSet
from mavros_msgs.msg import State, GlobalPositionTarget, ParamValue
from sensor_msgs.msg import BatteryState, NavSatFix

# ===============DroneBrain Class=========================================
# ========================================================================


class DroneBrain:
    # ===============Constructor Function=====================================
    # ========================================================================
    def __init__(self):

        # Initialize the UAV
        self.id = input("Drone ID #: ")
        self.uav_id = "uav" + str(self.id)

        rospy.init_node("controller" + str(self.id), anonymous=True)

        # Define constants and variables
        self.ABSOLUTE_SAFE_ZONE = 0.4 # Radius of safezone around UAV where collision avoidance will begin.
        self.VERTICAL_SAFE_ZONE = 0.4
        self.HORIZONAL_SAFE_ZONE = 1.5

        self.rate = rospy.Rate(50)  

        # Create required publsihers for control
        self.target_pos_pub = rospy.Publisher(self.uav_id + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.target_vel_pub = rospy.Publisher(self.uav_id + "/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        

        # Create required subcribers for information and control
        self.state_sub = rospy.Subscriber(self.uav_id + "/mavros/state", State, self.state_changed_callback)
        self.actual_pos_sub = rospy.Subscriber(self.uav_id + "/mavros/local_position/pose", PoseStamped, self.position_callback)
        self.actual_pos_sub = rospy.Subscriber(self.uav_id + "/mavros/global_position/global", NavSatFix, self.global_pos_changed_callback)
        self.actual_vel_sub = rospy.Subscriber(self.uav_id + "/mavros/local_position/velocity_local", TwistStamped, self.velocity_callback)
        self.battery_voltage_sub = rospy.Subscriber(self.uav_id + "/mavros/battery", BatteryState, self.battery_voltage_callback)
        self.network_state_sub = rospy.Subscriber("/network_map", Int32, self.network_state_changed_callback)  

        # Create Objects to pair with above pubs and subs    
        self.desired_velocity_object = TwistStamped()
        self.battery_voltage_object = BatteryState()   
        self.actual_global_pos_object = NavSatFix()
        self.neighbor_global_position_objects = {}
        self.desired_pos_object = PoseStamped()
        self.actual_pos_object = PoseStamped()
        self.actual_velocity_object = Twist()
        self.neighbor_velocity_objects = {}
        self.neighbor_position_objects = {}
        self.neighbor_global_pos_sub = {}
        self.num_nodes_in_swarm = Int32()
        self.mode_object = SetMode()
        self.neighbor_pos_sub = {}
        self.neighbor_vel_sub = {} 
        self.state = State()  

        self.currently_avoiding = False  

        self.count = 0

        print( self.get_param('COM_OBL_ACT'))
        self.set_failsafe_action(2)

    # ====================Callback Functions==================================
    # ========================================================================
    def neighbor_position_changed_callback(self, position_object, index):
        self.neighbor_position_objects[index] = position_object

    def neighbor_global_pos_changed_callback(self, position_object, index):

        self.neighbor_global_position_objects[index] = position_object
        # print self.neighbor_global_position_objects
        if(self.count > 100):
            self.count = 0
            for i in range(1, len(self.neighbor_global_position_objects)+2):
                if(str(i) in self.neighbor_global_position_objects.keys()):
                    absolute = str(self.get_absolute_distance_between(self.actual_global_pos_object,self.neighbor_global_position_objects[i]))
                    vert = str(self.actual_global_pos_object.altitude - self.neighbor_global_position_objects[i].altitude)
                    horiz = str(self.get_horizontal_distance_between(self.actual_global_pos_object,self.neighbor_global_position_objects[i]))
                    print "D"+str(i)+": Abs:" +absolute +" Hor:"+horiz + " Vert:"+vert
        else:
            self.count += 1

        if (self.is_impending_collision(self.neighbor_global_position_objects[index])): 
           self.currently_avoiding = True
           self.avoid_collision(self.neighbor_global_position_objects[index], index)

        else:
             self.currently_avoiding = False


    def neighbor_velocity_changed_callback(self, velocity_object, index):
        self.neighbor_velocity_objects[index] = velocity_object


    def network_state_changed_callback(self, new_num_nodes):
        self.num_nodes_in_swarm = new_num_nodes
        
        for i in range(1, self.num_nodes_in_swarm.data + 1):
            uav_name = "uav"+str(i)
            if (i != self.id):
                if not i in self.neighbor_position_objects.keys():

                    self.neighbor_position_objects[str(i)] = PoseStamped()
                    self.neighbor_global_position_objects[str(i)] = NavSatFix()
                    self.neighbor_velocity_objects[str(i)] = TwistStamped()
                    
                if not i in self.neighbor_position_objects.keys():
                    
                    self.neighbor_pos_sub[str(i)] = rospy.Subscriber(uav_name + "/mavros/local_position/pose", PoseStamped, self.neighbor_position_changed_callback, i )
                    self.neighbor_vel_sub[str(i)] = rospy.Subscriber(uav_name + "/mavros/local_position/velocity_local", TwistStamped, self.neighbor_velocity_changed_callback, i)
                    self.neighbor_global_pos_sub[str(i)] = rospy.Subscriber(uav_name + "/mavros/global_position/global", NavSatFix, self.neighbor_global_pos_changed_callback, i)

    def state_changed_callback(self, state_object):
        self.state = state_object
    
    def position_callback(self, position_object):
        self.actual_pos_object = position_object   
    
    def global_pos_changed_callback(self, position_object):
        self.actual_global_pos_object = position_object

    def velocity_callback(self, velocity_object):
        self.actual_velocity_object = velocity_object.twist
    
    def battery_voltage_callback(self, battery_voltage):
        self.battery_voltage_object.voltage = battery_voltage


    # ====================Collision Avoidance Functions=======================
    # ========================================================================
    def is_impending_collision(self, other_pos_object):
        # rospy.loginfo("DISTANCE FROM UAV " + str(self.get_absolute_distance_between(self.actual_pos_object, other_pos_object)))
        absolute_distance = self.get_absolute_distance_between(self.actual_global_pos_object, other_pos_object)
        horizontal_distance = self.get_horizontal_distance_between(self.actual_global_pos_object, other_pos_object)
        vertical_distance = abs(self.actual_global_pos_object.altitude - other_pos_object.altitude)
       
        if (absolute_distance < self.ABSOLUTE_SAFE_ZONE and horizontal_distance < self.HORIZONAL_SAFE_ZONE and vertical_distance < self.VERTICAL_SAFE_ZONE):
            print "Impending CRASH IN Vertical DIRECTION"
            return True
        elif (vertical_distance < 0.2 and horizontal_distance < 1):
            print "CRASH IN HORIZONTAL DIRECTION"
            return True
        elif (absolute_distance < self.ABSOLUTE_SAFE_ZONE):
            print "CRASH IN Any DIRECTION"
            return True
        else:
            return False

    def avoid_collision(self, other_global_pos_object, index):
        pass
        # We know the other UAV's Position and Velocity, how can we reliably avoid a collision??
        # self.set_velocity(vx, vy, vz)

    # =========Get Horizontal Distance Between Function=======================
    # Uses Haversine's Formula to calculate distance between two (lat, lon) pairs.
    # ========================================================================
    def get_horizontal_distance_between(self, first_pos_object, other_pos_object):
        
        R = 6371e3; # earth radius metres
        phi1 = math.radians(first_pos_object.latitude)
        phi2 = math.radians(other_pos_object.latitude)
        delta_phi = math.radians(other_pos_object.latitude-first_pos_object.latitude)
        delta_lambda = math.radians(other_pos_object.longitude-first_pos_object.longitude)

        a = math.sin(delta_phi/2) * math.sin(delta_phi/2) + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2) * math.sin(delta_lambda/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c

        return d


    # ==========Get Abosulute Distance Between Function=======================
    # Calulate horizontal distance thenn use pythagoras to add on the altitude  
    # separation distance since the haversine doesn't account for altitude.
    # ========================================================================
    def get_absolute_distance_between(self, first_pos_object, other_pos_object): 
        
        d = self.get_horizontal_distance_between(first_pos_object, other_pos_object)
        dh = abs(first_pos_object.altitude - other_pos_object.altitude)
        D = math.sqrt(d**2+dh**2)

        return D  
       

    # ====================Arming Function=====================================
    # ========================================================================
    def arm(self):
        cmd = CommandBool()
        cmd.value = True
        self.arming_service_client = rospy.ServiceProxy("uav" + str(self.id) + "/mavros/cmd/arming", CommandBool)
        while not self.arming_service_client(cmd.value).success is True:
            rospy.loginfo("UAV " + str(self.id) + " waiting to arm")

            if not self.currently_avoiding:
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
            
            return True
        else: 
            return False
        
    # ====================Set Velocity Function===============================
    # This function manually sets the velocity vector for the UAV it does not define the speed or max speed to a point!!
    # Can be used to set a trajectory for the UAV manually. I.e. go 180 backwards if you're about to crash
    # The max_vel is used to define max_speed for UAV, otherwise it goes as fast as it can by default
    # ========================================================================
    def set_velocity(self, vx, vy, vz):

        self.desired_velocity_object.header.stamp = rospy.Time.now()
        self.desired_velocity_object.twist.linear.x = vx
        self.desired_velocity_object.twist.linear.y = vy
        self.desired_velocity_object.twist.linear.z = vz
        
        self.target_vel_pub.publish(self.desired_velocity_object)
        # self.rate.sleep()


    # ====================Print Stats Function===============================
    # ========================================================================
    def get_stats(self):

        stats = {
            'id': self.id,
            'position': self.actual_pos_object,
            'velocity': self.actual_velocity_object,
            'battery_voltage': self.battery_voltage_object.voltage.voltage
        }
        return stats


    # ====================Set Velocity Function===============================
    # ========================================================================
    def set_mode(self, mode_string):

        self.set_mode_srv_client = rospy.ServiceProxy(self.uav_id + "/mavros/set_mode", SetMode)

        self.mode_object.custom_mode = mode_string
            
        rospy.loginfo("TRYING TO SET MODE " + mode_string)
        while not self.state.mode == mode_string:
            self.set_mode_srv_client(0, self.mode_object.custom_mode)
            # rospy.loginfo("MODE IS " + self.state.mode + " SHOULD BE " + mode_string)
            
            if not self.currently_avoiding:
                for i in range(100):
                    self.target_pos_pub.publish(self.desired_pos_object)
                    self.rate.sleep()
            
            self.check_and_rearm()

            self.rate.sleep()

        rospy.loginfo("MODE SUCCESSFULLY CHANGED")

    # ====================Takeoff Function====================================
    # ========================================================================
    def takeoff(self, alt):

        while not self.in_position():

            self.check_and_rearm()

            self.desired_pos_object.pose.position.z = alt

            if not self.currently_avoiding:
                self.target_pos_pub.publish(self.desired_pos_object)
                
            self.rate.sleep()
    
    # ====================check_and_rearm====================================
    # =======================================================================
    def check_and_rearm(self):

        if self.state.armed is not True:

                rospy.loginfo("UAV was disarmed. Attempting - Rearm.")
                self.arm()
                
    # ====================Move To Function===================================
    # =======================================================================
    def move_to(self, x_loc, y_loc, z_loc):
        
        self.desired_pos_object.pose.position.x = x_loc
        self.desired_pos_object.pose.position.y = y_loc
        self.desired_pos_object.pose.position.z = z_loc

        while not self.in_position():

            if(not self.currently_avoiding):
                self.target_pos_pub.publish(self.desired_pos_object)

            self.rate.sleep()

    # ====================Follow Function====================================
    # =======================================================================
    def follow(self, uav, x_offset, y_offset, z_offset, formation=None):

        uav_name = "uav" + str(uav)

        while not uav in self.neighbor_position_objects.keys():
           rospy.loginfo(uav_name + " not found in my network. Waiting...")
           for i in range(100):
               self.takeoff(1)

        x = self.desired_pos_object.pose.position.x
        y = self.desired_pos_object.pose.position.y
        z = self.desired_pos_object.pose.position.z

        x_new = self.neighbor_position_objects[uav].pose.position.x + x_offset
        y_new = self.neighbor_position_objects[uav].pose.position.y + y_offset
        z_new = self.neighbor_position_objects[uav].pose.position.z + z_offset
        
        self.desired_pos_object.pose.position.x = x_new
        self.desired_pos_object.pose.position.y = y_new
        self.desired_pos_object.pose.position.z = z_new
        
        self.check_and_rearm()
        if(not self.currently_avoiding):
            self.target_pos_pub.publish(self.desired_pos_object)

    def follow_callback(self, target_position_object):
        if self.id != 1:
            self.desired_pos_object = target_position_object

    # ====================Failsafe Change Call===============================
    # COM_OF_LOSS_T 	Time-out (in seconds) to wait when offboard connection is lost before triggering offboard lost failsafe (COM_OBL_ACT and COM_OBL_RC_ACT)
    # COM_OBL_ACT 	Mode to switch to if offboard control is lost when not connected to RC 
    #   (Values are - 0: Land, 1: Hold, 2: Return ).
    # COM_OBL_RC_ACT 	Mode to switch to if offboard control is lost while still connected to RC control 
    #   (Values are - 0: Position, 1: Altitude, 2: Manual, 3: Return , 4: Land).
    # =======================================================================
    def set_failsafe_action(self,action_mode):
        resp1 = self.set_param('COM_OBL_ACT', action_mode)
        resp2 = self.set_param('COM_OBL_RC_ACT', action_mode)

        if resp1:
            print("Successfully changed COM_OBL failsafe action")
        else:
            print "Failed to change failsafe."

        if resp2:
            print "Successfully changed COM_OBL_RC failsafe action"
        else:
            print "Failed to change failsafe."
    
    # ====================Service Calls======================================
    # Getting/Setting Parameters in PX4
    # rosservice call /param/set "param_id: 'MPC_LAND_SPEED'
    # =======================================================================
    def set_param(self, param, value):
        proper_value = ParamValue()
        proper_value.integer = value

        rospy.wait_for_service(self.uav_id + '/mavros/param/set')
        try:
            set_fcu_param = rospy.ServiceProxy(self.uav_id + '/mavros/param/set', ParamSet)
            resp = set_fcu_param(param, proper_value)
            return resp.success
        except rospy.ServiceException, e:
            print "Param Set Service call failed: %s"%e
    
    def get_param(self, param):
        print "GETTING " + param
        rospy.wait_for_service(self.uav_id + '/mavros/param/get')
        try:
            get_fcu_param = rospy.ServiceProxy(self.uav_id + '/mavros/param/get', ParamGet)
            resp = get_fcu_param(param)
            return resp.value
        except rospy.ServiceException, e:
            print "Param Set Service call failed: %s"%e

# ========================================================================
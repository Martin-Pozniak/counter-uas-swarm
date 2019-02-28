#!/usr/bin/env/ python

'''
# Author:           Martin Pozniak
# Creation Date:    2/23/19
# Last Edit Date:   2/27/19
# Description:      drone_brain.py initializes a new ROS node and contains all functions to control the UAV.
#                   It runs onboard the UAV.
#
# CHANGE THINGS TO GLOBAL REFERENCE. I THINK THE DISTANCE FUNCTION IS MESSED UP BECAUSE 
# EACH UAV IS REPORTING THEIR LOCAL FRAME POSITION WHICH ISN'T THE SAME BETWEEN ENITITES
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
        self.id = input("Drone ID #: ")
        self.uav_id = "uav" + str(self.id)

        rospy.init_node("controller" + str(self.id), anonymous=True)

        # Define constants and variables
        self.SAFE_ZONE = 1.53 # Radius of safezone around UAV where collision avoidance will begin.
        self.rate = rospy.Rate(50)  

        # Create required publsihers for control
        self.target_pos_pub = rospy.Publisher(self.uav_id + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.target_vel_pub = rospy.Publisher(self.uav_id + "/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        

        # Create required subcribers for information and control
        self.state_sub = rospy.Subscriber(self.uav_id + "/mavros/state", State, self.state_changed_callback)
        self.actual_pos_sub = rospy.Subscriber(self.uav_id + "/mavros/local_position/pose", PoseStamped, self.position_callback)
        self.actual_vel_sub = rospy.Subscriber(self.uav_id + "/mavros/local_position/velocity_local", TwistStamped, self.velocity_callback)
        self.battery_voltage_sub = rospy.Subscriber(self.uav_id + "/mavros/battery", BatteryState, self.battery_voltage_callback)
        self.network_state_sub = rospy.Subscriber("/swarm_node_count", Int32, self.network_state_changed_callback)
        self.neighbor_pos_sub = {}
        self.neighbor_global_pos_sub = {}
        self.neighbor_vel_sub = {}  # Initialized dynamically based on number of nodes in net

        #===========Attempy at sending and recieving global coords==========
        self.actual_pos_sub = rospy.Subscriber(self.uav_id + "/mavros/global_position/global", NavSatFix, self.global_pos_changed_callback)
        self.actual_global_pos_object = NavSatFix()
        #===================================================================

        # Create Objects to pair with above pubs and subs    
        self.actual_velocity_object = Twist() 
        self.battery_voltage_object = BatteryState()   
        self.desired_pos_object = PoseStamped()
        self.desired_velocity_object = TwistStamped()   # Notice not TwistStamped() we tunnel down to the twist object when receiving info from the callback
        self.actual_pos_object = PoseStamped()

        self.neighbor_position_objects = {}
        self.neighbor_global_position_objects = {}
        self.neighbor_velocity_objects = {}
        self.num_nodes_in_swarm = Int32()
        self.mode_object = SetMode()
        self.state = State()     

        #FIND A WAY TO WAIT UNTIL IT LISTENS FOR OTHER UAVS POSITIONS IN CASE YOU RUN INTO ISSUES WHERE OTHER UAV DATA ISNT BEING RECEIVED 

    # ====================Callback Functions==================================
    # ========================================================================
    def neighbor_position_changed_callback(self, position_object, index):
        self.neighbor_position_objects["uav"+str(index)] = position_object
        # print("uav" + str(index) + " POS Changed: X: "+ str(self.neighbor_position_objects["uav"+str(index)].pose.position.x))
        # rospy.loginfo("DISTANCE FROM UAV " + str(self.get_distance_between(self.actual_pos_object, position_object)))
        # if (self.is_impending_collision(self.neighbor_position_objects["uav"+str(index)])): #Check to make sure we should pass position object and not neighbor_position_objects[index]
        #     self.avoid_collision(self.neighbor_position_objects["uav"+str(index)])


    def neighbor_global_pos_changed_callback(self, position_object, index):
        self.neighbor_global_position_objects["uav"+str(index)] = position_object
        # print("uav" + str(index) + " POS Changed: lat: "+ str(self.neighbor_global_position_objects["uav"+str(index)].latitude))
        # rospy.loginfo("DISTANCE FROM UAV " + str(self.get_distance_between(self.actual_global_pos_object, position_object)))
        # self.get_distance_between(self.actual_global_pos_object, self.neighbor_global_position_objects["uav"+str(index)])

        if (self.is_impending_collision(self.neighbor_global_position_objects["uav"+str(index)])): #Check to make sure we should pass position object and not neighbor_position_objects[index]
           self.avoid_collision(self.neighbor_global_position_objects["uav"+str(index)])


    def neighbor_velocity_changed_callback(self, velocity_object, index):
        # print("Neighbor Velocity Changed")
        pass


    def network_state_changed_callback(self, new_num_nodes):
        self.num_nodes_in_swarm = new_num_nodes
        # rospy.loginfo("THERE ARE " + str(self.num_nodes_in_swarm.data) + " NODES IN THE SWARM")
        for i in range(1, self.num_nodes_in_swarm.data + 2):
            # Create subs for each node thats in the swarm (except outself) so we can monitor each of their positions           
            if (i != self.id):
                if not "uav"+str(i) in self.neighbor_position_objects.keys():
                    rospy.loginfo("Registering UAV " + str(i) + " in my network FROM CALLBACK")
                    self.neighbor_position_objects["uav"+str(i)] = PoseStamped()
                    self.neighbor_global_position_objects["uav"+str(i)] = NavSatFix()
                    self.neighbor_velocity_objects["uav"+str(i)] = TwistStamped()
                    
                if not i in self.neighbor_position_objects.keys():
                    rospy.loginfo("Registering UAV " + str(i) + " in my network FROM CALLBACK")
                    self.neighbor_pos_sub[i] = rospy.Subscriber("uav" + str(i) + "/mavros/local_position/pose", PoseStamped, self.neighbor_position_changed_callback, i )
                    self.neighbor_vel_sub[i] = rospy.Subscriber("uav" + str(i) + "/mavros/local_position/velocity_local", TwistStamped, self.neighbor_velocity_changed_callback, i)
                    self.neighbor_global_pos_sub[i] = rospy.Subscriber("uav" + str(i) + "/mavros/global_position/global", NavSatFix, self.neighbor_global_pos_changed_callback, i)
                    
                    #TRIPLE CHECK THERES NO SCOPING ISSUES CREATED HERE


    def state_changed_callback(self, state_object):
        self.state = state_object
    
    def position_callback(self, position_object):
        # rospy.loginfo("POSITION CALLBACK" + str(position_object.pose.position.z))
        self.actual_pos_object = position_object   
    
    def global_pos_changed_callback(self, position_object):
        # print "MY POS CHANGED"
        self.actual_global_pos_object = position_object
        for i in range(1, len(self.neighbor_global_position_objects)+1):
            if("uav"+str(i) in self.neighbor_global_position_objects.keys()):
                print "D"+str(i)+": " +str(self.get_distance_between(position_object,self.neighbor_global_position_objects["uav"+str(i)]) )

    def velocity_callback(self, velocity_object):
        # rospy.loginfo("Vx: " + str(velocity_object.twist.linear.x) + " Vy: " + str(velocity_object.twist.linear.y) + " Vz: " + str(velocity_object.twist.linear.z))
        self.actual_velocity_object = velocity_object.twist
    
    def battery_voltage_callback(self, battery_voltage):
        self.battery_voltage_object.voltage = battery_voltage

    # ====================Collision Avoidance Functions=======================
    # ========================================================================
    def is_impending_collision(self, other_pos_object):
        # rospy.loginfo("DISTANCE FROM UAV " + str(self.get_distance_between(self.actual_pos_object, other_pos_object)))
        if (self.get_distance_between(self.actual_global_pos_object, other_pos_object) < self.SAFE_ZONE ):  # If the distance between the UAVs is less than 1m in this case
            return True
        else:
            return False

    def avoid_collision(self, other_pos_object):
        #I dont think other_pos_object is being updated meaning we dont actually know where we are relative to the guy
        # rospy.loginfo("DISTANCE FROM UAV " + str(self.get_distance_between(self.actual_pos_object, other_pos_object)) + " Avoiding Collision")
        # while (self.get_distance_between(self.actual_global_pos_object, other_pos_object) < self.SAFE_ZONE):
        vx = -0.2 * abs(self.actual_velocity_object.linear.x)
        vy = -0.2 * abs(self.actual_velocity_object.linear.y)
        vz = -0.2 * abs(self.actual_velocity_object.linear.z)

        self.set_velocity(vx, vy, vz)
        for i in range(50):
            self.rate.sleep()

    # ====================Get Distance Between Function=======================
    # ========================================================================
    def get_distance_between(self, first_pos_object, other_pos_object): # Check whether to use first_position or the global self.actual_pos because  we may not get the most updated info
    
        R = 6371e3; # earth radius metres
        phi1 = math.radians(first_pos_object.latitude)
        phi2 = math.radians(other_pos_object.latitude)
        delta_phi = math.radians(other_pos_object.latitude-first_pos_object.latitude)
        delta_lambda = math.radians(other_pos_object.longitude-first_pos_object.longitude)

        a = math.sin(delta_phi/2) * math.sin(delta_phi/2) + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2) * math.sin(delta_lambda/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c

        dh = abs(first_pos_object.altitude - other_pos_object.altitude)

        D = math.sqrt(d**2+dh**2)

        return D  # D represents the haversine formula taking into account difference in altitude 
       

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
    def set_velocity(self, vx, vy, vz):
        # This function manually sets the velocity vector for the UAV it does not define the speed or max speed to a point!!
        # Can be used to set a trajectory for the UAV manually. I.e. go 180 backwards if you're about to crash
        # The max_vel is used to define max_speed for UAV, otherwise it goes as fast as it can by default

        self.desired_velocity_object.header.stamp = rospy.Time.now()
        self.desired_velocity_object.twist.linear.x = vx
        self.desired_velocity_object.twist.linear.y = vy
        self.desired_velocity_object.twist.linear.z = vz

        # self.desired_velocity_object.twist.angular.x = -
        # self.desired_velocity_object.twist.angular.y = -
        # self.desired_velocity_object.twist.angular.z = -

        
        self.target_vel_pub.publish(self.desired_velocity_object)
        self.rate.sleep()

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

        for i in range(100):
            self.target_pos_pub.publish(self.desired_pos_object)
            self.rate.sleep()
            
        rospy.loginfo("TRYING TO SET MODE " + mode_string)
        while not self.state.mode == mode_string:
            self.set_mode_srv_client(0, self.mode_object.custom_mode)
            rospy.loginfo("MODE IS " + self.state.mode + " SHOULD BE " + mode_string)

            self.check_and_rearm()

            for i in range(100):
                self.target_pos_pub.publish(self.desired_pos_object)
                self.rate.sleep()
            self.rate.sleep()

        rospy.loginfo("MODE SUCCESSFULLY CHANGED")

    # ================Dis-Arming Function=====================================
    # ========================================================================
    def disarm(self):
        pass

    # ====================Takeoff Function====================================
    # ========================================================================
    def takeoff(self, alt):
        self.check_and_rearm()
        
        self.desired_pos_object.pose.position.z = alt

        print str(self.desired_pos_object.pose.position.z) + " / " + str(self.actual_pos_object.pose.position.z)

        while not self.in_position():
            self.check_and_rearm()
            self.target_pos_pub.publish(self.desired_pos_object)
            self.rate.sleep()
    
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
    
    # ====================check_and_rearm===================================
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
            self.target_pos_pub.publish(self.desired_pos_object)
            self.rate.sleep()

    # ====================Follow Function====================================
    # =======================================================================
    def follow(self, uav, x_offset, y_offset, z_offset, formation=None):

        uav_name = "uav" + str(uav)
        while not uav_name in self.neighbor_position_objects.keys():
           rospy.loginfo(uav_name + " not found in my network. Waiting...")
           for i in range(100):
            self.takeoff(1)

        x = self.desired_pos_object.pose.position.x
        y = self.desired_pos_object.pose.position.y
        z = self.desired_pos_object.pose.position.z

        x_new = self.neighbor_position_objects[uav_name].pose.position.x + x_offset
        y_new = self.neighbor_position_objects[uav_name].pose.position.y + y_offset
        z_new = self.neighbor_position_objects[uav_name].pose.position.z + z_offset

        # print ( "F: " + uav_name + " X: " + str(round(self.actual_pos_object.pose.position.x, 0)) + "/" + str(round(x_new, 0)) + 
        #         " @ " + str(round(self.actual_velocity_object.linear.x)) +
        #         "m/s y: " + str(round(self.actual_pos_object.pose.position.y, 0)) + "/" + str(round(y_new, 0)) + 
        #         " @ " + str(round(self.actual_velocity_object.linear.y)) + 
        #         "m/s Alt: " + str(round(self.actual_pos_object.pose.position.z, 0)) + "/" + str(round(z_new, 0)) + 
        #         " @ " + str(round(self.actual_velocity_object.linear.z,1)) + "m/s" )
        
        self.desired_pos_object.pose.position.x = x_new
        self.desired_pos_object.pose.position.y = y_new
        self.desired_pos_object.pose.position.z = z_new
        
        self.check_and_rearm()
        self.target_pos_pub.publish(self.desired_pos_object)
        # self.rate.sleep()

    def follow_callback(self, target_position_object):
        if self.id != 1:
            self.desired_pos_object = target_position_object

# ========================================================================

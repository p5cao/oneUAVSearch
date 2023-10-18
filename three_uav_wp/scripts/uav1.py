#!/usr/bin/env python3
"""
This script does waypoints following for three UAVs
The waypoints come from Muhan's MPC search path code
1- Set the offboard mode for three UAVs 
2- Arming on and take off
3- Do waypoint following along the search path
4- Return to home position and land

Notes:
Before entering Offboard mode, you must have already started streaming setpoints.
 Otherwise the mode switch will be rejected.
"""

# libraries and messages 
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import pprint
import networkx as nx
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# from ayush.second_step_on_vertex_visit import second_step_on_vertex_visit
# from ayush.initialize_graph import Robot,Vertex, build_graph, find_shortest_path
# from ayush.first_step_on_vertex_visit import Id,what_to_do_if_next_node_known,first_step_on_arriving_at_vertex
# from ayush.get_incidence_matrix import get_incidence_matrix
# from ayush.order_matrix import completed,out,unexplored
pp = pprint.PrettyPrinter(indent=8)

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
new_pose = PoseStamped()

def state_cb(state):
    global current_state
    current_state = state

def position_cb(Pose):
    global new_pose
    new_pose = Pose

local_pos_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)

rospy.Subscriber('uav1/mavros/state', State, state_cb)
rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, position_cb)

#state_sub = rospy.Subscriber(mavros.get_topic('uav1','state'), State, state_cb)
arming_client = rospy.ServiceProxy('uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
set_mode_client = rospy.ServiceProxy('uav1/mavros/set_mode', mavros_msgs.srv.SetMode)

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 0

threshold = 0.1

def read_data(filename):
    with open(filename) as f:
         lines = f.readlines()

    x_strings, y_strings, psi_strings = lines[0].split('  '), lines[1].split('  '), lines[2].split('  ')
    x_list, y_list, psi_list, list_of_setpoints = [],[],[],[]
    for x in x_strings:
         try: 
         	x_float = float(x) -10.0
         except:
            continue
         x_list.append(x_float)
    for y in y_strings:
         try: 
         	y_float = float(y) - 10.0
         except:
            continue
         y_list.append(y_float)
    for psi in psi_strings:
         try: 
         	psi_float = float(psi)  - np.pi*5/4
         except:
            continue
         psi_list.append(psi_float)

    
    for i in range(0,100,5):
         list_of_setpoints.append(x_list[i])
         list_of_setpoints.append(y_list[i])
         list_of_setpoints.append(10.0)
         list_of_setpoints.append(psi_list[i])
    return list_of_setpoints

def drone_reached(xdata, ydata, zdata):
    if abs(new_pose.pose.position.x - xdata) <= threshold and abs(new_pose.pose.position.y - ydata) <= threshold \
         and abs(new_pose.pose.position.z - zdata): # <= threshold and abs(new_pose.pose.orientation.z - psidata) <= threshold:
        return True        

def position_control(list_of_setpoints0):
    
    # list_of_setpoints0, list_of_setpoints1, list_of_setpoints2, list_of_setpoints3 = algorithm()
    # list_of_setpoints0 = list([-5.0, -5.0, 10.0, -5.0, -4.0, 10.0, -5.0, -3.0, 10.0, -5.0, -2.0, 10.0, -5.0, -1.0, 10.0])
    
    # num_of_points = len(list_of_setpoints0)//3
    num_of_points = len(list_of_setpoints0)//4
    print(list_of_setpoints0)
    rospy.init_node('offb_node1', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()
    print(num_of_points)
    for i in range(num_of_points):
        # print(i)
        last_request = rospy.get_rostime()

        while not rospy.is_shutdown() and not drone_reached(list_of_setpoints0[4*i] ,list_of_setpoints0[4*i+1],
                                                            list_of_setpoints0[4*i+2]):
            now = rospy.get_rostime()
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(4.)):
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now 
            else:
                if not current_state.armed and (now - last_request > rospy.Duration(4.)):
                    arming_client(True)
                    last_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != current_state.armed:
                rospy.loginfo("UAV1 Vehicle armed: %r" % current_state.armed)
            if prev_state.mode != current_state.mode: 
                rospy.loginfo("UAV1 Current mode: %s" % current_state.mode)
            prev_state = current_state
            
            # new_x,new_y,new_z = 
            # Update timestamp and publish pose 
            #pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = list_of_setpoints0[4*i] 
            pose.pose.position.y = list_of_setpoints0[4*i+1] 
            pose.pose.position.z = list_of_setpoints0[4*i+2]
            pose.pose.orientation.z = list_of_setpoints0[4*i+3]
            local_pos_pub.publish(pose)
            #print('here!!')
            rate.sleep()

if __name__ == '__main__':
	list_of_setpoints1 = read_data('/home/sza817353/catkin_ws/src/three_uav_wp/trial2_uav2_waypoints_ubd.txt')
	try:
        #print('D')
		position_control(list_of_setpoints1)
	except rospy.ROSInterruptException:
		pass

	
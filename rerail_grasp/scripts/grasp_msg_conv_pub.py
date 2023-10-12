#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import math
import time
import threading
import sys
import tf2_ros
import argparse as ap        
import numpy as np
import os

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


### Rail imports
from rerail_grasp.msg import grasp_target_msg, Bool2DArray, Bool1DArray, object_ellipse_msg
from util import grasp_target_bridge
import pickle as pkl

# read data from file
def read_data_from_file(file_name):
    with open(file_name, 'rb') as f:
        data = pkl.load(f)
    return data

# class grasp_target_bridge:
#     def __init__(self):
#         pass

#     def flatten_2d_array(self,array):
#         # RAIL code
#         flattened_array = []
#         for row in array:
#             for el in row:
#                 flattened_array.append(el)
#         return(flattened_array)

#     def unflatten_2d_array(self,flattened_array,shape=(2,2)):
#         # RAIL code
#         array = []
#         for i in range(shape[0]):
#             row = []
#             for j in range(shape[1]):
#                 row.append(flattened_array[i*shape[1]+j])
#             array.append(tuple(row))
#         return(tuple(array))

#     def get_Bool2DArray(self,bool_array):
#         array_Bool1DArray = []
#         for row in bool_array:
#             msg_1d = Bool1DArray()
#             msg_1d.data_1d = list(row)
#             array_Bool1DArray.append(msg_1d)
#         msg_2d = Bool2DArray()
#         # print("OK "*50)
#         msg_2d.data_2d = array_Bool1DArray
#         return(msg_2d)
    
#     def get_inv_Bool2DArray(self,msg_2d):
#         array_bool = []
#         for row in msg_2d.data_2d:
#             array_bool.append(row.data_1d)
#         return(np.array(array_bool))

#     def get_ellipse_msg(self,ellipse):
#         ellipse_msg = object_ellipse_msg()
#         ellipse_msg.centroid = ellipse['centroid']
#         ellipse_msg.minor_axis = self.flatten_2d_array(ellipse['minor']['axis'])
#         ellipse_msg.minor_length = ellipse['minor']['length']
#         ellipse_msg.major_axis = self.flatten_2d_array(ellipse['major']['axis'])
#         # print('ellipse_major_axis',ellipse['major']['axis'])
#         # print('ellipse_mag_major_axis',ellipse_msg.major_axis)

#         ellipse_msg.major_length = ellipse['major']['length']
#         # print(ellipse['minor'].keys())
#         # ellipse_msg.minor_ang_rad = ellipse['minor']['ang_rad']
#         ellipse_msg.major_ang_rad = ellipse['major']['ang_rad']

#         return(ellipse_msg)
    
#     def get_ellipse(self,ellipse_msg):
#         ellipse = {}
#         ellipse['centroid'] = ellipse_msg.centroid
#         ellipse['minor'] = {}
#         ellipse['minor']['axis'] = ellipse_msg.minor_axis
#         ellipse['minor']['length'] = ellipse_msg.minor_length
#         ellipse['major'] = {}
#         ellipse['major']['axis'] = ellipse_msg.major_axis
#         ellipse['major']['length'] = ellipse_msg.major_length
#         # ellipse['minor']['ang_rad'] = ellipse_msg.minor_ang_rad
#         ellipse['major']['ang_rad'] = ellipse_msg.major_ang_rad
#         return(ellipse)

#     def to_ros_msg(self,grasp_target):
#         grasp_target_ros_msg = grasp_target_msg()
#         grasp_target_ros_msg.location_xy_pix = grasp_target['location_xy_pix']
#         grasp_target_ros_msg.elongated = grasp_target['elongated']
#         grasp_target_ros_msg.width_pix = grasp_target['width_pix']
#         grasp_target_ros_msg.width_m = grasp_target['width_m']
    
#         grasp_target_ros_msg.aperture_axis_pix = self.flatten_2d_array(grasp_target['aperture_axis_pix'])
#         grasp_target_ros_msg.long_axis_pix = self.flatten_2d_array(grasp_target['long_axis_pix'])
#         grasp_target_ros_msg.location_above_surface_m = grasp_target['location_above_surface_m']
#         grasp_target_ros_msg.location_z_pix = grasp_target['location_z_pix']
#         grasp_target_ros_msg.object_max_height_above_surface_m = grasp_target['object_max_height_above_surface_m']
        
#         grasp_target_ros_msg.surface_convex_hull_mask = self.get_Bool2DArray(grasp_target['surface_convex_hull_mask'])
#         grasp_target_ros_msg.object_selector = self.get_Bool2DArray(grasp_target['object_selector'])
#         grasp_target_ros_msg.object_ellipse = self.get_ellipse_msg(grasp_target['object_ellipse'])
#         return(grasp_target_ros_msg)

#     def from_ros_msg(self,grasp_target_ros_msg):
#         grasp_target = {}
#         grasp_target['location_xy_pix'] = grasp_target_ros_msg.location_xy_pix
#         grasp_target['elongated'] = grasp_target_ros_msg.elongated
#         grasp_target['width_pix'] = grasp_target_ros_msg.width_pix
#         grasp_target['width_m'] = grasp_target_ros_msg.width_m
#         grasp_target['aperture_axis_pix'] = grasp_target_ros_msg.aperture_axis_pix
#         grasp_target['long_axis_pix'] = grasp_target_ros_msg.long_axis_pix
#         grasp_target['location_above_surface_m'] = grasp_target_ros_msg.location_above_surface_m
#         grasp_target['location_z_pix'] = grasp_target_ros_msg.location_z_pix
#         grasp_target['object_max_height_above_surface_m'] = grasp_target_ros_msg.object_max_height_above_surface_m
#         grasp_target['surface_convex_hull_mask'] = self.get_inv_Bool2DArray(grasp_target_ros_msg.surface_convex_hull_mask)
#         grasp_target['object_selector'] = self.get_inv_Bool2DArray(grasp_target_ros_msg.object_selector)
#         grasp_target['object_ellipse'] = self.get_ellipse(grasp_target_ros_msg.object_ellipse)
#         return(grasp_target)


# grasp_target = read_data_from_file('grasp_target.pkl')
# # print(grasp_target['object_ellipse'])
# for key in grasp_target['object_ellipse'].keys():
#     print(key,grasp_target['object_ellipse'][key])
# gt_bridge = grasp_target_bridge()
# grasp_target_ros_msg = gt_bridge.to_ros_msg(grasp_target)   

# grasp_target_converted = gt_bridge.from_ros_msg(grasp_target_ros_msg)

class GraspPublisher():
    def __init__(self) -> None:
        self.grasp_target_pub = rospy.Publisher('/grasp_target', grasp_target_msg, queue_size=1)
        grasp_target = read_data_from_file('grasp_target.pkl')
        gt_bridge = grasp_target_bridge()
        grasp_target_ros_msg = gt_bridge.to_ros_msg(grasp_target)   
        self.grasp_target = grasp_target_ros_msg
        # self.grasp_target_pub.publish(self.grasp_target)
    def publish(self):
        self.grasp_target_pub.publish(self.grasp_target)

def main():
    rospy.init_node('grasp_target_pub_node', anonymous=True)
    grasp_target_pub = GraspPublisher()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        grasp_target_pub.publish()
        rate.sleep()

if __name__ == '__main__':
    main()
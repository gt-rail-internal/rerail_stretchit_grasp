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



grasp_target = read_data_from_file('grasp_target.pkl')
# print(grasp_target['object_ellipse'])
for key in grasp_target['object_ellipse'].keys():
    print(key,grasp_target['object_ellipse'][key])
gt_bridge = grasp_target_bridge()
grasp_target_ros_msg = gt_bridge.to_ros_msg(grasp_target)   

grasp_target_converted = gt_bridge.from_ros_msg(grasp_target_ros_msg)

class GraspTargetSub():
    def __init__(self):
        self.grasp_target = None
        self.grasp_target_sub = rospy.Subscriber('/grasp_target',grasp_target_msg,self.grasp_target_cb)
        self.gt_bridge = grasp_target_bridge()
    
    def grasp_target_cb(self,msg):
        self.grasp_target = self.gt_bridge.from_ros_msg(msg)
        print('grasp_target',self.grasp_target)

def main():
    rospy.init_node('grasp_target_sub_node', anonymous=True)
    grasp_target_sub = GraspTargetSub()
    rospy.spin()

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray
import numpy as np
from rail_manipulation_msgs.msg import SegmentedObjectList
class rail_detic_bridge:
    def __init__(self):
        rospy.init_node('rail_detic_bridge', anonymous=True)
        self.mask_x_sub = rospy.Subscriber("mask_x", Int32MultiArray, self.mask_x_callback)
        self.mask_y_sub = rospy.Subscriber("mask_y", Int32MultiArray, self.mask_y_callback)
        self.rail_seg_sub = rospy.Subscriber('rail_segmentation', SegmentedObjectList, self.rail_seg_callback)
        self.mask_x_sub
        self.mask_y_sub
        self.rail_seg_sub
        self.mask_x = None
        self.mask_y = None

    def rail_seg_callback(self, data):
        

    def mask_x_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.mask_x = np.array(data.data)
    def mask_y_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.mask_y = np.array(data.data)
    def get_mask(self):
        self.mask = np.array([self.mask_x, self.mask_y])
        return self.mask

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
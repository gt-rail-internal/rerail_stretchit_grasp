#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('get_tf_transform')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform('link_grasp_center', 'link_gripper', rospy.Time())
            rospy.loginfo('Transform from link1 to link2: %s', transform)
            print("Transform is: ", transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo('Transform not found')
        rate.sleep()

#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
# from rail_manipulation_msgs.srv import SegmentObjects, SegmentObjectsRequest, SegmentObjectsResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

def segment_client():
    print("Waiting for service")
    rospy.wait_for_service('/rail_segmentation/segment')
    print("Service found")
    try:
        segment_serv = rospy.ServiceProxy('/rail_segmentation/segment', Empty)
        resp1 = segment_serv()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Requesting segmentation")
    print(segment_client())
    print("Segmentation complete")
#!/usr/bin/env python3

import rospy
from rail_manipulation_msgs.srv import SegmentObjects, SegmentObjectsRequest, SegmentObjectsResponse
# from your_package.msg import YourMessageType
# from your_package.srv import YourServiceType
from stretch_fetch_grasp_bridge.srv import StretchSegmentation, StretchSegmentationRequest, StretchSegmentationResponse

class SegmentationDummyNode(object):
    def __init__(self):
        rospy.init_node('segmentation_dummy_node')

        # Create the service
        self.segmentation_service = rospy.Service('/stretch_segmentation/segment_objects', StretchSegmentation, self.segmentation_service_callback)
        # creating client
        rospy.loginfo("Waiting for service /rail_segmentation/segment_objects")
        rospy.wait_for_service('/rail_segmentation/segment_objects')
        self.segmentation_client = rospy.ServiceProxy('/rail_segmentation/segment_objects', SegmentObjects)
    def segmentation_service_callback(self, req):
        # Create the response
        resp = StretchSegmentationResponse()
        # Call the service
        objects = self.segmentation_client()

        # Add the segmented objects to the response
        # resp.segmented_objects = [YourMessageType()]
        if(len(objects.segmented_objects.objects) == 0):
            # print("no objects found")
            resp.success = False
            return resp
        resp.segmented_point_cloud = objects.segmented_objects.objects[0].point_cloud
        resp.success = True
        # Return the response
        return resp

def main():
    print("Segmentation dummy node")
    node = SegmentationDummyNode()
    # Shutdown the ROS node
    rospy.spin()

if __name__ == '__main__':
    main()

# import rospy

# import sys
# import rospy
# from rail_manipulation_msgs.srv import SegmentObjects, SegmentObjectsRequest, SegmentObjectsResponse

# def segment_obj_client():
#     print("Waiting for service")
#     rospy.wait_for_service('/rail_segmentation/segment_objects')
#     try:
#         get_segmented_objects = rospy.ServiceProxy('/rail_segmentation/segment_objects', SegmentObjects)
#         resp1 = get_segmented_objects()
#         cloud = resp1.segmented_objects.objects[0].point_cloud
#         print(type(cloud))
#         return 
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)



# if __name__ == "__main__":
#     segment_obj_client()

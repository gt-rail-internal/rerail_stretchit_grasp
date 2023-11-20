#!/usr/bin/env python3

import rospy
from rail_manipulation_msgs.srv import SegmentObjects, SegmentObjectsRequest, SegmentObjectsResponse
from segmentation.srv import Object_detection
# from your_package.msg import YourMessageType
# from your_package.srv import YourServiceType
from stretch_fetch_grasp_bridge.srv import StretchSegmentation, StretchSegmentationRequest, StretchSegmentationResponse
from sensor_msgs.msg import Image
import numpy as np

class SegmentationNode(object):
    def __init__(self):
        rospy.init_node('segmentation_node')

        # creating subscriber
        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.img_callback)

        # Create the service
        self.segmentation_service = rospy.Service('/stretch_segmentation/segment_objects', StretchSegmentation, self.segmentation_service_callback)
        # creating client
        rospy.loginfo("Waiting for service /rail_segmentation/segment_objects")
        rospy.wait_for_service('/rail_segmentation/segment_objects')
        self.segmentation_client = rospy.ServiceProxy('/rail_segmentation/segment_objects', SegmentObjects)
        rospy.loginfo("Got service /rail_segmentation/segment_objects")
        rospy.loginfo("Waiting for service /object_detection")
        rospy.wait_for_service('/object_detection')
        self.detic_client = rospy.ServiceProxy('object_detection',Object_detection)
        rospy.loginfo("Got service /object_detection")
    
    def img_callback(self, data):
        self.img = data

    def find_object_index(self, rail_objects,detic_detection):
        bounding_coords = np.array(detic_detection.bounding_box)
        center_ooi = np.array([np.mean(bounding_coords[0:2]),np.mean(bounding_coords[2:4])])
        obj_centers = np.array([])
        for i in range(len(rail_objects.objects)):
            obj = rail_objects.objects[i]
            # Trasform the point to the target frame
            point_transformed = self.transform_point_to_frame(obj.centroid, 'base_link', 'camera_color_optical_frame')
            pixel_coords = self.project_to_image_plane(point_transformed.point)
            obj_centers = np.append(obj_centers, pixel_coords)
        # differnce between the center of the object of interest and the center of the rail
        self.diff = obj_centers - self.center_ooi
        closest_index = np.argmin(np.abs(self.diff))
        return closest_index
        
    def segmentation_service_callback(self, req):
        # Create the response
        resp = StretchSegmentationResponse()
        # Call the service
        objects = self.segmentation_client()
        # Call the service
        detic_detection = self.detic_client(req.object_name, self.img)

        object_index = self.find_object_index(objects,detic_detection)

        # Add the segmented objects to the response
        # resp.segmented_objects = [YourMessageType()]
        resp.segmented_point_cloud = objects.segmented_objects.objects[object_index].point_cloud
        # Return the response
        return resp

def main():
    print("Segmentation dummy node")
    node = SegmentationNode()
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

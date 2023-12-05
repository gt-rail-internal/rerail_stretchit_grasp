#!/usr/bin/env python3

import rospy
from rail_manipulation_msgs.srv import SegmentObjects, SegmentObjectsRequest, SegmentObjectsResponse
from segmentation.srv import Object_detection
# from your_package.msg import YourMessageType
# from your_package.srv import YourServiceType
from stretch_fetch_grasp_bridge.srv import StretchSegmentation, StretchSegmentationRequest, StretchSegmentationResponse
from sensor_msgs.msg import Image
import numpy as np

import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge

class SegmentationNode(object):
    def __init__(self):
        rospy.init_node('segmentation_node')
        self.bridge = CvBridge() 

        # creating subscriber
        self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.img_callback)
        self.point_sub = rospy.Subscriber('/camera/depth/color/points',PointCloud2,self.point_cloud_callback)
        # sub to camera info
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # creating publisher
        self.segmented_cloud_pub = rospy.Publisher('/segmented_cloud', PointCloud2, queue_size=10)


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
    def camera_info_callback(self, data):
        self.camera_info = data
    def img_callback(self, data):
        self.img = data
    def point_cloud_callback(self,data):
        self.point_cloud = data

    def transform_point_to_frame(self, point, from_frame, to_frame, timeout=rospy.Duration(1.0)):
        """
        Transform a point from one frame to another using tf2_ros.

        :param point: A PointStamped object representing the point in the original frame.
        :param from_frame: The frame in which the point is currently.
        :param to_frame: The target frame to which the point should be transformed.
        :param timeout: The time to wait for the transformation to be available.
        
        :return: A PointStamped object of the point in the target frame.
        """
        # Initialize a tf2 buffer and listener if not already done
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Wait for the transform to become available
        try:
            tf_buffer.can_transform(to_frame, from_frame, rospy.Time(0), timeout)
            point_Stamped = tf2_geometry_msgs.PointStamped()
            point_Stamped.header.frame_id = from_frame
            point_Stamped.point = point

            # Transform the point to the target frame
            point_transformed = tf_buffer.transform(point_Stamped, to_frame, timeout)
            return point_transformed

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('Error transforming point from %s to %s: %s', from_frame, to_frame, e)
            return None
    
    def segment_point_cloud_v1(self,segmented_pixels, point_cloud_msg, camera_info_msg):
        """
        Segments a point cloud based on segmented pixel coordinates and camera information.

        :param segmented_pixels: List of (x, y) tuples representing segmented pixel coordinates.
        :param point_cloud_msg: ROS message of type PointCloud2.
        :param camera_info_msg: ROS message of type CameraInfo.
        :return: Segmented point cloud as a PointCloud2 message.
        """
        # Convert camera information ROS message to a usable format
        K = np.array(camera_info_msg.K).reshape(3, 3)
        P = np.array(camera_info_msg.P).reshape(3, 4)

        # Convert the PointCloud2 message to a list of points
        cloud_points = pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
        print("cloud_points:",cloud_points) 
        print("point_cloud_msg:",point_cloud_msg.fields)
        # Project pixel coordinates to 3D space and segment the point cloud
        segmented_points = []
        # print("segmented_pixels:",segmented_pixels)
        for (u, v) in segmented_pixels:
            # print("hello")
            # Convert to normalized device coordinates
            u_n = (u - K[0, 2]) / K[0, 0]
            v_n = (v - K[1, 2]) / K[1, 1]

            # Project to 3D space using depth information
            for point in cloud_points:
                x, y, z = point[:3]
                if x / z == u_n and y / z == v_n:
                    print("found point")
                    segmented_points.append((x, y, z))
                    break

        # Create a new PointCloud2 message for the segmented points
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = point_cloud_msg.header.frame_id  # Use the same frame_id as the input point cloud

        fields = [pc2.PointField(name=n, offset=i*4, datatype=pc2.PointField.FLOAT32, count=1)
                for i, n in enumerate('xyz')]

        segmented_cloud_msg = pc2.create_cloud(header, fields, segmented_points)
        
        print(segmented_cloud_msg)
        print(segmented_points)
        print(type(segmented_cloud_msg))
        return segmented_cloud_msg
    # def segment_point_cloud(self,segmented_pixels, point_cloud_msg, camera_info_msg):
    #     """
    #     Segments a point cloud based on segmented pixel coordinates, camera information, and includes RGB data.

    #     :param segmented_pixels: List of (x, y) tuples representing segmented pixel coordinates.
    #     :param point_cloud_msg: ROS message of type PointCloud2.
    #     :param camera_info_msg: ROS message of type CameraInfo.
    #     :return: Segmented point cloud as a PointCloud2 message with RGB data.
    #     """
    #     # Check if segmented pixels list is empty
    #     if not segmented_pixels:
    #         print("No segmented pixels provided.")
    #         return None

    #     # Convert camera information ROS message to a usable format
    #     K = np.array(camera_info_msg.K).reshape(3, 3)

    #     # Convert the PointCloud2 message to a list of points with RGB data
    #     cloud_points = pc2.read_points(point_cloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))

    #     print("cloud_points:",cloud_points)
    #     # Project pixel coordinates to 3D space and segment the point cloud with RGB data
    #     segmented_points = []
    #     for (u, v) in segmented_pixels:
    #         # Convert to normalized device coordinates
    #         u_n = (u - K[0, 2]) / K[0, 0]
    #         v_n = (v - K[1, 2]) / K[1, 1]

    #         found_match = False
    #         for point in cloud_points:
    #             # print('hi')
    #             x, y, z, rgb = point
    #             # Assuming a small threshold for matching
    #             threshold = 0.3  # Adjust as needed
    #             if abs(x / z - u_n) < threshold and abs(y / z - v_n) < threshold:
    #                 # print('hi')
    #                 segmented_points.append([x, y, z, rgb])
    #                 found_match = True
    #                 break
    #         if not found_match:
    #             pass
    #             #print(f"No match found for pixel: ({u}, {v})")

    #     print('length of segmented points:',len(segmented_points))
    #     print('lenght of segmented_pixels:',len(segmented_pixels))
    #     if not segmented_points:
    #         print("Segmented point cloud is empty.")
    #         return None

    #     # Create a new PointCloud2 message for the segmented points with RGB data
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = point_cloud_msg.header.frame_id

    #     fields = [pc2.PointField(name=n, offset=i*4, datatype=pc2.PointField.FLOAT32, count=1)
    #             for i, n in enumerate('xyz')]
    
    #     fields.append(pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1))
    #     print(fields)
    #     print("## Original fields ##")
    #     print(point_cloud_msg.fields)
    #     fields = point_cloud_msg.fields
    #     print("segmented_points[:10]:",segmented_points[0])
    #     segmented_cloud_msg = pc2.create_cloud(header, fields, segmented_points)
    #     print(segmented_cloud_msg)
    #     self.segmented_cloud_pub.publish(segmented_cloud_msg)
    #     return segmented_cloud_msg
    def project_to_image_plane(self, point_3d, fx=905.608154296875, fy=903.4915771484375, cx=644.8792114257812, cy=361.4921569824219):
        """
        Project a 3D point in the camera frame to 2D pixel coordinates.

        :param point_3d: A tuple (X, Y, Z) representing the 3D point in camera frame.
        :param fx: Focal length in x direction.
        :param fy: Focal length in y direction.
        :param cx: Optical center x coordinate.
        :param cy: Optical center y coordinate.

        :return: A tuple (u, v) representing the 2D pixel coordinates.
        """
        #X, Y, Z = point_3d
        X = point_3d.x
        Y = point_3d.y
        Z = point_3d.z
        if Z == 0:
            raise ValueError("Z coordinate is zero, cannot project to image plane.")
        u = (fx * X) / Z + cx
        v = (fy * Y) / Z + cy
        return int(round(u)), int(round(v))  # Pixel coordinates are typically expressed as integers

    def find_object_index(self, rail_objects,detic_detection):
        bounding_coords = np.array(detic_detection.bounding_box.data)
        print("bounding_coords:")
        print(bounding_coords)
        center_ooi = np.array([np.mean(bounding_coords[0:2]),np.mean(bounding_coords[2:4])])
        # obj_centers = np.array([])
        obj_centers = np.zeros((len(rail_objects.objects),2))
        for i in range(len(rail_objects.objects)):
            obj = rail_objects.objects[i]
            # Trasform the point to the target frame
            point_transformed = self.transform_point_to_frame(obj.centroid, 'base_link', 'camera_color_optical_frame')
            pixel_coords = self.project_to_image_plane(point_transformed.point)
            obj_centers[i,:] = pixel_coords
            print("obj_centers_shape:")
            print(obj_centers.shape)
        # differnce between the center of the object of interest and the center of the rail
        # obj_centers = obj_centers[:,np.newaxis]
        # center_ooi = center_ooi[np.newaxis,:]
        self.diff = obj_centers - center_ooi[np.newaxis,:]
        print("center_ooi shape:")
        print(center_ooi.shape)
        print("diff shape:")
        print(self.diff.shape)
        self.diff = np.abs(self.diff)
        print("diff_abs:")
        print(self.diff)
        self.diff = np.sum(self.diff,axis=1)
        print("diff:")
        print(self.diff.shape)
        self.debug_img(center_ooi,obj_centers)
        closest_index = np.argmin(self.diff)
        return closest_index
    def debug_img(self,coi,obj_centers):
        cv_img = self.bridge.imgmsg_to_cv2(self.img,"bgr8")
        
        cv2.imwrite("/home/hello-robot/cv_img.png",cv_img)
        # img = cv2.rotate(cv_img,cv2.ROTATE_90_CLOCKWISE)
        # img = cv2.flip(img,1)
        img = cv_img
        # center of interest
        img = cv2.circle(img,(int(coi[0]),int(coi[1])), 5, (0,0,255), -1)
        # object centers
        print("obj_centers:")
        print(obj_centers)
        print("obj_centers_shape:   ")
        print(obj_centers.shape)
        for i in range(len(obj_centers)):
            img = cv2.circle(img,(int(obj_centers[i][0]),int(obj_centers[i][1])), 5, (0,255,0), -1)
        cv2.imwrite("/home/hello-robot/debug_img.png",img)
        print("saved debug image")



        
    def segmentation_service_callback(self, req):
        rospy.loginfo("RAIL seg + DETIC segmentation ")
        # Create the response
        resp = StretchSegmentationResponse()
        # Call the service
        objects = self.segmentation_client()
        objects = objects.segmented_objects
        print("Got objects from RAIL seg:")
        print(type(objects))
        # raise NotImplementedError
        # print("no of objects:",len(objects.objects))
        # Call the service
        rospy.loginfo("Calling detic service")
        detic_detection = self.detic_client(req.object_name, self.img)
        rospy.loginfo("Got Detic Detection")
        # pixels = []
        # print("len of detic_detection.masks_x.data:",len(detic_detection.masks_x.data))
        # raise NotImplementedError
        # for x,y in zip(detic_detection.masks_x.data, detic_detection.masks_y.data):
        #     pixels.append((y,x))
        # segmented_cloud = self.segment_point_cloud(pixels,self.point_cloud,self.camera_info)
        # print(objects.objects[0].point_cloud)
        rospy.loginfo("Got response from detic ")
        object_index = self.find_object_index(objects,detic_detection)
        rospy.loginfo("object_index:"+str(object_index))
        print("#"*50)


        # Add the segmented objects to the response
        # resp.segmented_objects = [YourMessageType()]
        resp.segmented_point_cloud =  objects.objects[object_index].point_cloud #segmented_cloud
        resp.success = True
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

#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray,Int16
import numpy as np
from rail_manipulation_msgs.msg import SegmentedObjectList
import tf2_ros
import tf2_geometry_msgs

class rail_detic_bridge:
    def __init__(self):
        rospy.init_node('rail_detic_bridge', anonymous=True)
        self.detic_seg_sub = rospy.Subscriber('detic_segmentation', Int32MultiArray, self.detic_callback)
        self.rail_seg_sub = rospy.Subscriber('rail_segmentation', SegmentedObjectList, self.rail_seg_callback)
        self.obj_pub = rospy.Publisher('object_index',Int16 , queue_size=10)
        self.mask_x_sub
        self.mask_y_sub
        self.rail_seg_sub
        self.mask_x = None
        self.mask_y = None
        self.center_ooi = None # center of object of interest
    def detic_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.bounding_coords = np.array(data.data)
        self.center_ooi = np.array([np.mean(self.bounding_coords[0:2]),np.mean(self.bounding_coords[2:4])])
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


    def rail_seg_callback(self, data):
        obj_centers = np.array([])
        for i in range(len(data.objects)):
            obj = data.objects[i]
            # Trasform the point to the target frame
            point_transformed = self.transform_point_to_frame(obj.centroid, 'base_link', 'camera_color_optical_frame')
            pixel_coords = self.project_to_image_plane(point_transformed.point)
            obj_centers = np.append(obj_centers, pixel_coords)
        # differnce between the center of the object of interest and the center of the rail
        self.diff = obj_centers - self.center_ooi
        closest_index = np.argmin(np.abs(self.diff))
        self.obj_pub.publish(closest_index)

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
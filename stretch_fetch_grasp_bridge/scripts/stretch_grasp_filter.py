#!/usr/bin/env python
# Credits to ChatGPT
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
import tf.transformations as tf_transform
import numpy as np


class stretch_grasp_filter:
    def __init__(self):
        # configs
        self.filter_angle = 10 # degrees
        
        rospy.init_node('pose_array_listener', anonymous=True)

        # subscribers and publishers
        self.all_grasp_sub = rospy.Subscriber('/test_grasp_suggestion/all_grasp_poses', PoseArray, self.pose_array_callback)
        self.filter_grasp_pub = rospy.Publisher('/stretch_grasp/filtered_grasp_poses', PoseArray, queue_size=10)
        self.last_pos_req_sub = rospy.Subscriber('/stretch_grasp/last_pos_req',Int16, self.last_pos_req_sub_callback)
        self.single_grasp_pub = rospy.Publisher('/stretch_grasp/single_grasp_pose', PoseStamped, queue_size=10)
        self.last_pose_array = None
        # self.stretch_grasp_pose_service = rospy.Service('/stretch_grasp/get_grasp_pose', PoseArray, self.pose_array_callback)
    def debug_pose(self,pose):
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        point_in_coord1 = self.point_in_coordinate1(quat)
        print("Point in coord1 is: ", point_in_coord1)
        ang = self.angle_between_vectors(point_in_coord1, [0,0,1])
        deg_ang = np.rad2deg(ang)
        rospy.loginfo("#"*10)
        rospy.loginfo("Quaternion is: %f, %f, %f, %f", quat[0], quat[1], quat[2], quat[3])  
        rospy.loginfo("Angle is: %f", deg_ang)
        rospy.loginfo("Point in coord1 is: %f, %f, %f", point_in_coord1[0], point_in_coord1[1], point_in_coord1[2])
        rospy.loginfo("Point in coord2 is: %f, %f, %f", 1, 0, 0)
        
    def last_pos_req_sub_callback(self,data):
        rospy.loginfo("Received last_pos_req: %d", data.data)
        if self.last_pose_array is None:
            rospy.loginfo("last_pose_array is None")
            return
        if data.data < len(self.last_pose_array.poses):
            rospy.loginfo("Publishing single grasp pose")
            single_pose = PoseStamped()
            single_pose.header = self.last_pose_array.header
            single_pose.pose = self.last_pose_array.poses[data.data]
            self.debug_pose(single_pose.pose)

            self.single_grasp_pub.publish(single_pose)
            
        else:
            rospy.loginfo("last_pos_req is out of bounds")
            return
        

    def angle_between_vectors(self,A, B):
        """
        Returns the angle (in radians) between vectors A and B.

        Parameters:
        - A: First vector.
        - B: Second vector.

        Returns:
        - Angle in radians between the two vectors.
        """
        
        dot_product = np.dot(A, B)
        magnitude_A = np.linalg.norm(A)
        magnitude_B = np.linalg.norm(B)
        
        cos_theta = dot_product / (magnitude_A * magnitude_B)
        
        # Ensure the value lies between -1 and 1 to avoid numerical errors
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        
        angle = np.arccos(cos_theta)
        return angle

    def point_in_coordinate1(self,quat, point_in_coord2=[1,0,0]):
        """
        Returns the coordinates of a point (originally in coordinate system 2) in coordinate system 1 using tf.

        Parameters:
        - quat: A quaternion (x, y, z, w) representing the rotation from coordinate system 1 to 2.
        - point_in_coord2: The point's coordinates in coordinate system 2 (default is [1,0,0]).

        Returns:
        - The point's coordinates in coordinate system 1.
        """

        # Convert the quaternion to a transformation matrix
        matrix = tf_transform.quaternion_matrix(quat)

        # We only need the 3x3 rotation matrix part
        rotation_matrix = matrix[:3, :3]
        point_in_coord1 = np.dot(rotation_matrix, point_in_coord2)

        # Compute the inverse rotation matrix
        # inv_rotation_matrix = np.linalg.inv(rotation_matrix)

        # Multiply the point with the inverse matrix to get the coordinates in coordinate system 1
        # point_in_coord1 = np.dot(inv_rotation_matrix, point_in_coord2)

        return point_in_coord1

    def pose_array_callback(self,data):
        rospy.loginfo("Received PoseArray with %d poses.", len(data.poses))
        filtered_poses = PoseArray()
        filtered_poses.header = data.header
        print("Header is: ", data.header)
        for i, pose in enumerate(data.poses):
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            point_in_coord1 = self.point_in_coordinate1(quat)
            ang = self.angle_between_vectors(point_in_coord1, [0,0,1])
            deg_ang = np.rad2deg(ang)
            if((deg_ang> 90-self.filter_angle) and (deg_ang<90+self.filter_angle)): # can make this a parameter
                print("Angle is: ", deg_ang)
                filtered_poses.poses.append(pose)
        self.filter_grasp_pub.publish(filtered_poses)
        self.last_pose_array = filtered_poses
        rospy.loginfo("Published PoseArray with %d poses.", len(filtered_poses.poses))


if __name__ == '__main__':
    try:
        stretch_grasp_filter()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
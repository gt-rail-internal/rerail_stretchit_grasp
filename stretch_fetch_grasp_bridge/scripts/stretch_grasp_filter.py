#!/usr/bin/env python
# Credits to ChatGPT
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32, String
import tf.transformations as tf_transform
from stretch_fetch_grasp_bridge.srv import StretchGraspPosev2, StretchGraspPosev2Response
from rail_manipulation_msgs.srv import SuggestGrasps, SuggestGraspsRequest, SuggestGraspsResponse
import numpy as np


class StretchGraspFilter:
    def __init__(self):
        rospy.init_node('stretch_grasp_filter', anonymous=True)
        
        
        # params
        self.filter_angle_buffer = rospy.get_param('/stretch_grasp/filter_angle_buffer', 10) #10 # (in degrees) the buffer around the filter_angle
        self.displace_distance = rospy.get_param('/stretch_grasp/displace_distance', 0.03) #0.03 # (in meters) the distance to displace the grasp along x-axis of the grasp frame
        self.type_of_grasps = rospy.get_param('/stretch_grasp/filtering_type', 'all')
        #        possible values:
        #           "frontal": grasps with the gripper pointing parallel to the ground plane
        #           "top_down": grasps with the gripper pointing directly down
        #           "lateral": grasps that have the antipodal point in a line parallel to the ground plane.
        #           "all": all grasps without any filtering
        
        
        # Publishers
        self.filter_grasp_pub = rospy.Publisher('/stretch_grasp/filtered_grasp_poses', PoseArray, queue_size=10)
        self.single_grasp_pub = rospy.Publisher('/stretch_grasp/single_grasp_pose', PoseStamped, queue_size=10)
        self.debug_grasp_pub = rospy.Publisher('/stretch_grasp/debug_grasp_pose', PoseStamped, queue_size=10)

        # Services
        ## server
        self.stretch_grasp_service = rospy.Service('/stretch_grasp_pose_suggester', StretchGraspPosev2, self.stretch_grasp_service_callback)

        ## client
        rospy.loginfo("Waiting for fetch suggestion service")
        rospy.wait_for_service('/suggester/suggest_grasps')
        self.fetch_grasp_client = rospy.ServiceProxy('/suggester/suggest_grasps', SuggestGrasps)
        rospy.loginfo("Connected to fetch suggestion service")
        
    
    def stretch_grasp_service_callback(self,request):
        '''
        Callback function for the stretch_grasp_pose_suggester service. 
        This function takes in a point cloud and returns a PoseStamped message type containing the pose of the best grasp.

        Parameters:
        - request: A StretchGraspPosev2Request message type containing the point cloud.

        Returns:
        - A StretchGraspPosev2Response message type containing the pose of the best grasp.
        '''
        
        rospy.logdebug("Received grasp request")
        # updating params
        self.displace_distance = rospy.get_param('/stretch_grasp/displace_distance', 0.03) #0.03 # (in meters) the distance to displace the grasp along x-axis of the grasp frame
        
        # Fetch Grasp
        fetch_request = SuggestGraspsRequest()
        fetch_request.cloud = request.point_cloud
        fetch_grasp_resp = self.fetch_grasp_client(fetch_request)
        grasp_list = fetch_grasp_resp.grasp_list

        # Filter Grasp
        filtered_grasps = self.filter_grasps(grasp_list)
        selected_grasp = filtered_grasps.poses[0]
        
        # Transform Grasp for viable Stretch Grasp
        aligned_grasp = self.align_new_pose_frame(selected_grasp)
        displaced_grasp = self.displace_pose([self.displace_distance,0.0,0.0],aligned_grasp)


        resp = StretchGraspPosev2Response()
        resp.grasp_pose.header.frame_id = 'base_link'
        resp.grasp_pose.header.stamp = rospy.Time.now()
        
        resp.grasp_pose.pose = aligned_grasp
        self.debug_grasp_pub.publish(resp.grasp_pose)

        resp.grasp_pose.pose = displaced_grasp
        self.single_grasp_pub.publish(resp.grasp_pose)
        resp.success = True
        return resp

    def filter_grasps(self,grasp_list)->PoseArray:
        """
        Filters the grasps based on the type_of_grasps parameter.

        Parameters:
        - grasp_list: A PoseArray message type containing the list of grasps to be filtered.

        Returns:
        - A PoseArray message type containing the filtered grasps.
        """
        self.type_of_grasps = rospy.get_param('filtering_type', 'all')
        self.filter_angle_buffer = rospy.get_param('/stretch_grasp/filter_angle_buffer', 10) #10 # (in degrees) the buffer around the filter_angle

        rospy.logdebug("type_of_grasps is: " + str(self.type_of_grasps))
        rospy.logdebug("Received PoseArray with %d poses.", len(grasp_list.poses))
        if(self.type_of_grasps == 'all'):
            self.filter_grasp_pub.publish(grasp_list)
            self.filtered_pose_array = grasp_list
            rospy.logdebug("Published PoseArray with %d poses.", len(grasp_list.poses))
            return(grasp_list)
        if(self.type_of_grasps == 'frontal'):
            self.filter_angle = 90 # degrees
            axis_of_interest = [1,0,0]
            self.filter_angle_buffer = 10
        elif(self.type_of_grasps == 'top_down'):
            self.filter_angle = 180
            axis_of_interest = [1,0,0]
            self.filter_angle_buffer = 20
        elif(self.type_of_grasps == 'lateral'):
            self.filter_angle = 90
            axis_of_interest = [0,1,0]
            self.filter_angle_buffer = 10
        else:
            rospy.logerr("Unknown type of grasps for filtering. Defaulting to all grasps")
            self.filter_grasp_pub.publish(grasp_list)
            self.filtered_pose_array = grasp_list
        
        filtered_poses = PoseArray()
        filtered_poses.header = grasp_list.header
        
        for i, pose in enumerate(grasp_list.poses):
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            point_in_coord1 = self.point_in_coordinate1(quat,axis_of_interest) # returns the vector axis_of_interest vector in the base_link frame.
            ang = self.angle_between_vectors(point_in_coord1, [0,0,1])
            deg_ang = np.rad2deg(ang)
          
            if((deg_ang> self.filter_angle-self.filter_angle_buffer) and (deg_ang<self.filter_angle+self.filter_angle_buffer)): # can make this a parameter
                filtered_poses.poses.append(pose)
        self.filter_grasp_pub.publish(filtered_poses)
        self.filtered_pose_array = filtered_poses 
        rospy.logdebug("Published PoseArray with %d poses.", len(filtered_poses.poses))
        return(filtered_poses)
        
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
    
    def quaternion_to_rotation_matrix(self,quaternion): # Credit to ChatGPT4
        """
        Converts a quaternion into a rotation matrix.
        
        Args:
        quaternion (tuple): A quaternion represented as (x, y, z, w).

        Returns:
        np.array: A 3x3 rotation matrix.
        """
        x, y, z, w = quaternion
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
    
    def rotation_matrix_to_quaternion(self,rot_matrix): # Credit to ChatGPT4
        """
        Converts a rotation matrix to a quaternion.

        Args:
        rot_matrix (np.array): A 3x3 rotation matrix.

        Returns:
        tuple: A quaternion (x, y, z, w).
        """
        m00, m01, m02 = rot_matrix[0, 0], rot_matrix[0, 1], rot_matrix[0, 2]
        m10, m11, m12 = rot_matrix[1, 0], rot_matrix[1, 1], rot_matrix[1, 2]
        m20, m21, m22 = rot_matrix[2, 0], rot_matrix[2, 1], rot_matrix[2, 2]

        tr = m00 + m11 + m22

        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2  # S=4*qw
            qw = 0.25 * S
            qx = (m21 - m12) / S
            qy = (m02 - m20) / S
            qz = (m10 - m01) / S
        elif (m00 > m11) and (m00 > m22):
            S = np.sqrt(1.0 + m00 - m11 - m22) * 2  # S=4*qx
            qw = (m21 - m12) / S
            qx = 0.25 * S
            qy = (m01 + m10) / S
            qz = (m02 + m20) / S
        elif m11 > m22:
            S = np.sqrt(1.0 + m11 - m00 - m22) * 2  # S=4*qy
            qw = (m02 - m20) / S
            qx = (m01 + m10) / S
            qy = 0.25 * S
            qz = (m12 + m21) / S
        else:
            S = np.sqrt(1.0 + m22 - m00 - m11) * 2  # S=4*qz
            qw = (m10 - m01) / S
            qx = (m02 + m20) / S
            qy = (m12 + m21) / S
            qz = 0.25 * S

        return (qx, qy, qz, qw)
    def align_new_pose_frame(self,pose): # Credit to ChatGPT4
        """
        Creates a new pose frame with the same y-axis as pose_frame and the same orientation
        of the z-axis as the base_link frame's.

        Args:
        pose (Pose): A Pose message type.

        Returns:
        Pose: A new pose which is aligned to the z-axis of the base_link frame and has the similar y-axis as the original pose.
        """
        # Extract the rotation matrix from the original pose's quaternion
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrix = self.quaternion_to_rotation_matrix(quaternion)

        # Extract the y-axis from the original pose's rotation matrix
        y_axis = rotation_matrix[:, 1]

        # The z-axis aligned with the base_link frame's z-axis
        z_axis = np.array([0, 0, 1])

        # Compute the x-axis using the cross product to ensure orthogonality
        x_axis = np.cross(y_axis, z_axis)
        
        # Ensure the x-axis points in the same direction as the original x-axis
        if(x_axis[1]>0):
            x_axis = -x_axis

        # Ensure the y-axis is orthogonal to the new x and z axes
        y_axis = np.cross(z_axis, x_axis)

        # Recompose the rotation matrix
        new_rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        # Convert the rotation matrix back to a quaternion, if needed
        new_orientation = self.rotation_matrix_to_quaternion(new_rotation_matrix)

        # Create new pose
        new_pose = Pose()
        
        new_pose.position = pose.position
        new_pose.orientation.x = new_orientation[0]
        new_pose.orientation.y = new_orientation[1]
        new_pose.orientation.z = new_orientation[2]
        new_pose.orientation.w = new_orientation[3]

        return new_pose

    def displace_pose(self,displacement_grasp_frame,pose):
        """
        Displaces a pose by the specified amount in the grasp frame.

        Args:
        displacement_grasp_frame (list): A list containing the x, y, z displacement in the grasp frame.
        pose (Pose): A ros Pose message type to be displaces.

        Returns:
        PoseStamped: The displaced pose.
        """
        rotation_matrix = self.quaternion_to_rotation_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        x_axis = rotation_matrix[:, 0]
        y_axis = rotation_matrix[:, 1]
        z_axis = rotation_matrix[:, 2]
        new_pose = Pose()
        new_pose.position.x = pose.position.x + displacement_grasp_frame[0]*x_axis[0] + displacement_grasp_frame[1]*y_axis[0] + displacement_grasp_frame[2]*z_axis[0]
        new_pose.position.y = pose.position.y + displacement_grasp_frame[0]*x_axis[1] + displacement_grasp_frame[1]*y_axis[1] + displacement_grasp_frame[2]*z_axis[1]
        new_pose.position.z = pose.position.z + displacement_grasp_frame[0]*x_axis[2] + displacement_grasp_frame[1]*y_axis[2] + displacement_grasp_frame[2]*z_axis[2]
        new_pose.orientation = pose.orientation
        return new_pose
if __name__ == '__main__':
    try:
        StretchGraspFilter()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
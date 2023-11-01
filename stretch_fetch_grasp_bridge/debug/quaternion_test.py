import tf.transformations as tf_transform
import numpy as np
def point_in_coordinate1(quat, point_in_coord2=[1,0,0]):
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
        print(matrix)

        # We only need the 3x3 rotation matrix part
        rotation_matrix = matrix[:3, :3]
        point_in_coord1 = np.dot(rotation_matrix, point_in_coord2)


        # Compute the inverse rotation matrix
        # inv_rotation_matrix = np.linalg.inv(rotation_matrix)
        # Multiply the point with the inverse matrix to get the coordinates in coordinate system 1
        # point_in_coord1 = np.dot(inv_rotation_matrix, point_in_coord2)

        return point_in_coord1

qaut = [0,0,0.7071068,0.7071068]
point = point_in_coordinate1(qaut)
print(point)
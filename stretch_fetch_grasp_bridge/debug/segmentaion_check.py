#!/usr/bin/env python
import rospy
from rail_manipulation_msgs.msg import SegmentedObjectList
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf2_ros
import tf2_geometry_msgs

def transform_point_to_frame(point, from_frame, to_frame, timeout=rospy.Duration(1.0)):
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
def project_to_image_plane(point_3d, fx=905.608154296875, fy=903.4915771484375, cx=644.8792114257812, cy=361.4921569824219):
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

def highlight_indices_in_image(indices):
    """
    Takes an image and a 1D array of indices, and sets the corresponding pixels in the image to black.

    :param image: A 2D numpy array representing the image.
    :param indices: A 1D numpy array or list of indices of the image to be highlighted.
    
    :return: A 2D numpy array representing the modified image.
    """
    image = cv2.imread('img.jpeg')
    # Calculate image width and height
    # height, width = image.shape[:2]
    width ,height = image.shape[:2]

    

    # Convert 1D indices to 2D pixel coordinates
    # coords_2d = [(index % width, index // width) for index in indices]
    coords_2d = [(index // width,index % width ) for index in indices]

    # Create a copy of the image to avoid modifying the original
    highlighted_image = image.copy()

    # Set the corresponding pixels to black
    for coord in coords_2d:
        highlighted_image[coord[1], coord[0]] = 0 if image.ndim == 2 else (0, 0, 0)
    cv2.imwrite('highlightened_img.jpeg', highlighted_image)
    return highlighted_image
def show_image(img_msg):
    # Create a CvBridge instance
    bridge = CvBridge()
    
    try:
        # Convert the ROS Image message to a cv2 image
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return
    
    # Display the cv2 image
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)  # Waits for a key press for 3 milliseconds
def unflatten_index(flat_index, img_shape):
    row_size, col_size = img_shape[:2]
    row_index = flat_index // row_size
    col_index = flat_index % row_size
    return row_index, col_index
    # return row_index, col_index

def plot_circle(img, index, radius=10, color=(0, 0, 255), thickness=2):
    # row, col = unflatten_index(index, img.shape)
    row,col = index[0]  , index[1]
    img[row, col] = [255, 0, 0]
    cv2.circle(img, (col, row), radius, color, thickness)
    return img
def segment_from_indexes(indexes):
    img = cv2.imread('img.jpeg')
    for index in indexes:
        img = plot_circle(img, index)
    #save img
    cv2.imwrite('segmented_img.jpeg', img)
def print_obj_data(obj):
    # print('name:',obj.name)
    # print('rgb:',obj.rgb)
    # print('cielab:',obj.cielab)
    # print('model_id',obj.model_id)
    # print('orientation',obj.orientation)
    # print('grasps',obj.grasps)
    # print('recognized',obj.recognized)
    # print('confidence',obj.confidence)
    # print('bounding_volume',obj.bounding_volume)
    # # print('marker',obj.marker)
    # img = obj.image
    # print('image',type(img))
    # img_shp = np.shape(img)
    # print('image shape:',img_shp)
    # point_cloud = obj.point_cloud
    # pc_shp = np.shape(point_cloud)
    # print('point_cloud shape:',pc_shp)
    # img_indices = obj.image_indices
    # im_ind_shp = np.shape(img_indices)
    # print('image_indices shape:',im_ind_shp)
    # print('image_indices',type(obj.image_indices))
    # show img
    print('img:',obj.image_indices[:10])
    print('width',obj.width)
    print('height',obj.height)
    print('center',obj.center)
def plot_center(obj):
    img = cv2.imread('img.jpeg')
    center = (int(obj.center.x*img.shape[0]+img.shape[0]/2),int(obj.center.y*img.shape[1]+img.shape[1]/2))
    print("CENTER:",center)
    img = plot_circle(img, center)
    #save img
    cv2.imwrite('center_img.jpeg', img)
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(len(data.objects))
    for i in range(len(data.objects)):
        obj = data.objects[i]
        print("running callback")
        highlight_indices_in_image(obj.image_indices)
        print('center:',obj.center)
        print('Centroid:',obj.centroid)

        # Trasform the point to the target frame
        point_transformed = transform_point_to_frame(obj.centroid, 'base_link', 'camera_color_optical_frame')
        pixel_coords = project_to_image_plane(point_transformed.point)
        print('pixel_coords:',pixel_coords)
        img = cv2.imread('img_flipped.jpeg')
        img_2 = plot_circle(img, pixel_coords)
        cv2.imwrite('center_img_'+str(i)+'.jpeg', img_2)
        print('point_transformed:',point_transformed)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("rail_segmentation/segmented_objects", SegmentedObjectList, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
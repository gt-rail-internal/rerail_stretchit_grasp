#!/usr/bin/env python
import rospy
from rail_manipulation_msgs.msg import SegmentedObjectList
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

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
    if len(data.objects) > 0:
        print_obj_data(data.objects[0])
        print_obj_data(data.objects[1])
        plot_center(data.objects[0])
        # segment_from_indexes(data.objects[0].image_indices)
        # print('name:',data.objects[0].name)
        # print('rgb:',data.objects[0].rgb)
        # print('cielab:',data.objects[0].cielab)
        # print('model_id',data.objects[0].model_id)
        # print('orientation',data.objects[0].orientation)
        # print('grasps',data.objects[0].grasps)
        # print('recognized',data.objects[0].recognized)
        # print('confidence',data.objects[0].confidence)
        # print('bounding_volume',data.objects[0].bounding_volume)
        # # print('marker',data.objects[0].marker)
        # img = data.objects[0].image
        # print('image',type(img))
        # img_shp = np.shape(img)
        # print('image shape:',img_shp)
        # point_cloud = data.objects[0].point_cloud
        # pc_shp = np.shape(point_cloud)
        # print('point_cloud shape:',pc_shp)
        # img_indices = data.objects[0].image_indices
        # im_ind_shp = np.shape(img_indices)
        # print('image_indices shape:',im_ind_shp)
        # print('image_indices',type(data.objects[1].image_indices))
        # # show img
        # print('img:',img_indices[:10])
        # print('width',data.objects[1].width)
        # print('height',data.objects[1].height)  
        # # show_image(img)
    
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
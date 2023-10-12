from rerail_grasp.msg import grasp_target_msg, Bool2DArray, Bool1DArray, object_ellipse_msg
import numpy as np
import pickle as pkl
class grasp_target_bridge:
    def __init__(self):
        pass

    def flatten_2d_array(self,array):
        # RAIL code
        flattened_array = []
        for row in array:
            for el in row:
                flattened_array.append(el)
        return(flattened_array)

    def unflatten_2d_array(self,flattened_array,shape=(2,2)):
        # RAIL code
        array = []
        for i in range(shape[0]):
            row = []
            for j in range(shape[1]):
                row.append(flattened_array[i*shape[1]+j])
            array.append(tuple(row))
        return(tuple(array))

    def get_Bool2DArray(self,bool_array):
        array_Bool1DArray = []
        for row in bool_array:
            msg_1d = Bool1DArray()
            msg_1d.data_1d = list(row)
            array_Bool1DArray.append(msg_1d)
        msg_2d = Bool2DArray()
        # print("OK "*50)
        msg_2d.data_2d = array_Bool1DArray
        return(msg_2d)
    
    def get_inv_Bool2DArray(self,msg_2d):
        array_bool = []
        for row in msg_2d.data_2d:
            array_bool.append(row.data_1d)
        return(np.array(array_bool))

    def get_ellipse_msg(self,ellipse):
        ellipse_msg = object_ellipse_msg()
        ellipse_msg.centroid = ellipse['centroid']
        ellipse_msg.minor_axis = self.flatten_2d_array(ellipse['minor']['axis'])
        ellipse_msg.minor_length = ellipse['minor']['length']
        ellipse_msg.major_axis = self.flatten_2d_array(ellipse['major']['axis'])
        # print('ellipse_major_axis',ellipse['major']['axis'])
        # print('ellipse_mag_major_axis',ellipse_msg.major_axis)

        ellipse_msg.major_length = ellipse['major']['length']
        # print(ellipse['minor'].keys())
        # ellipse_msg.minor_ang_rad = ellipse['minor']['ang_rad']
        ellipse_msg.major_ang_rad = ellipse['major']['ang_rad']

        return(ellipse_msg)
    
    def get_ellipse(self,ellipse_msg):
        ellipse = {}
        ellipse['centroid'] = ellipse_msg.centroid
        ellipse['minor'] = {}
        ellipse['minor']['axis'] = self.unflatten_2d_array(ellipse_msg.minor_axis)
        ellipse['minor']['length'] = ellipse_msg.minor_length
        ellipse['major'] = {}
        ellipse['major']['axis'] = self.unflatten_2d_array(ellipse_msg.major_axis)
        ellipse['major']['length'] = ellipse_msg.major_length
        # ellipse['minor']['ang_rad'] = ellipse_msg.minor_ang_rad
        ellipse['major']['ang_rad'] = ellipse_msg.major_ang_rad
        return(ellipse)

    def to_ros_msg(self,grasp_target):
        grasp_target_ros_msg = grasp_target_msg()
        grasp_target_ros_msg.location_xy_pix = grasp_target['location_xy_pix']
        grasp_target_ros_msg.elongated = grasp_target['elongated']
        grasp_target_ros_msg.width_pix = grasp_target['width_pix']
        grasp_target_ros_msg.width_m = grasp_target['width_m']
    
        grasp_target_ros_msg.aperture_axis_pix = self.flatten_2d_array(grasp_target['aperture_axis_pix'])
        grasp_target_ros_msg.long_axis_pix = self.flatten_2d_array(grasp_target['long_axis_pix'])
        grasp_target_ros_msg.location_above_surface_m = grasp_target['location_above_surface_m']
        grasp_target_ros_msg.location_z_pix = grasp_target['location_z_pix']
        grasp_target_ros_msg.object_max_height_above_surface_m = grasp_target['object_max_height_above_surface_m']
        
        grasp_target_ros_msg.surface_convex_hull_mask = self.get_Bool2DArray(grasp_target['surface_convex_hull_mask'])
        grasp_target_ros_msg.object_selector = self.get_Bool2DArray(grasp_target['object_selector'])
        grasp_target_ros_msg.object_ellipse = self.get_ellipse_msg(grasp_target['object_ellipse'])
        return(grasp_target_ros_msg)

    def from_ros_msg(self,grasp_target_ros_msg):
        grasp_target = {}
        grasp_target['location_xy_pix'] = grasp_target_ros_msg.location_xy_pix
        grasp_target['elongated'] = grasp_target_ros_msg.elongated
        grasp_target['width_pix'] = grasp_target_ros_msg.width_pix
        grasp_target['width_m'] = grasp_target_ros_msg.width_m
        grasp_target['aperture_axis_pix'] = self.unflatten_2d_array(grasp_target_ros_msg.aperture_axis_pix)
        grasp_target['long_axis_pix'] = self.unflatten_2d_array(grasp_target_ros_msg.long_axis_pix)
        grasp_target['location_above_surface_m'] = grasp_target_ros_msg.location_above_surface_m
        grasp_target['location_z_pix'] = grasp_target_ros_msg.location_z_pix
        grasp_target['object_max_height_above_surface_m'] = grasp_target_ros_msg.object_max_height_above_surface_m
        grasp_target['surface_convex_hull_mask'] = self.get_inv_Bool2DArray(grasp_target_ros_msg.surface_convex_hull_mask)
        grasp_target['object_selector'] = self.get_inv_Bool2DArray(grasp_target_ros_msg.object_selector)
        grasp_target['object_ellipse'] = self.get_ellipse(grasp_target_ros_msg.object_ellipse)
        return(grasp_target)
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from stretch_fetch_grasp_bridge.srv import StretchGraspPose, StretchGraspPoseResponse
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, PoseArray
class my_node:
    def __init__(self):
        rospy.init_node('my_node', anonymous=True)
        self.stretch_grasp_suggestor_service = rospy.Service('/stretch_grasp_pose_suggester', StretchGraspPose, self.stretch_grasp_service_callback)
        # self.my_topic = rospy.Subscriber('/', String, self.my_topic_callback)

        rospy.loginfo("Waiting for segmentation service")
        rospy.wait_for_service('/rail_segmentation/segment')
        rospy.loginfo("Segmentation service found")
        
        
        self.segmentation_client = rospy.ServiceProxy('/rail_segmentation/segment', Empty)
        
        self.grasp_list_req_pub = rospy.Publisher('/test_grasp_suggestion/grasp_object_heuristic_all_pose', Int32, queue_size=10)
        self.single_grasp_pose_req_pub = rospy.Publisher('/stretch_grasp/last_pos_req', Int32, queue_size=10)
        
        self.grasp_pose_sub = rospy.Subscriber('/stretch_grasp/single_grasp_pose', PoseStamped, self.grasp_pose_callback)
        self.grasp_unfiltered_list_sub = rospy.Subscriber('/test_grasp_suggestion/all_grasp_poses', PoseArray, self.grasp_unfiltered_list_callback)
        self.recovery_sub = rospy.Subscriber('/stretch_grasp/recovery', String, self.recovery_callback)
        
        self.object_index_sub = rospy.Subscriber('/object_index', Int32, self.object_index_callback)

        self.target_grasp_pose = None
        self.unfiltered_list = None
        self.object_index = None
    def object_index_callback(self,msg):
        self.object_index = msg.data
    def recovery_callback(self,msg):
        self.target_grasp_pose = PoseStamped()
        self.response.success = False

        rospy.loginfo("Recovery message received: %s", msg.data)
    
    def grasp_unfiltered_list_callback(self,msg):
        self.unfiltered_list = msg
        rospy.loginfo("Unfiltered list received")
    def grasp_pose_callback(self,msg):
        rospy.loginfo("Grasp pose received")
        self.target_grasp_pose = msg
        rospy.loginfo("Message received: %s", msg.pose)
        # self.last_string = msg.data
        # rospy.loginfo("Message received: %s", msg.data)
    def stretch_grasp_service_callback(self,request):
        
        self.last_string = None
        rospy.loginfo("Service called")
        self.response = StretchGraspPoseResponse()
        self.response.success = True

        # call segmentation service
        rospy.loginfo("Calling segmentation service")
        res = self.segmentation_client()
        rospy.loginfo("Segmentation service returned")
        self.object_index = None
        rospy.loginfo("Waiting for object index")
        while(self.object_index is None):
            rospy.sleep(0.1)
        #self.object_index = request.segment_no
        self.unfiltered_list = None
        self.grasp_list_req_pub.publish(self.object_index)
        while(self.unfiltered_list is None):
            rospy.sleep(0.1)
        self.target_grasp_pose = None
        rospy.loginfo("Sending request for grasp pose")
        self.single_grasp_pose_req_pub.publish(request.rank_no)
        while(self.target_grasp_pose is None):
            rospy.sleep(0.1)
        rospy.loginfo("Returning from service")

        
        
        # while(self.last_string is None):
        #     #spin rospy
        #     rospy.sleep(0.1)
        self.response.grasp_pose = self.target_grasp_pose
        # response.message = self.last_string
        return self.response

    

if __name__ == '__main__':
    node = my_node()
    rospy.spin()
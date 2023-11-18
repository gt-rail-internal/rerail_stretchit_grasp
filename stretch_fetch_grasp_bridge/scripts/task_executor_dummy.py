
import rospy
# from your_package.msg import YourMessageType
# from your_package.srv import YourServiceType
from stretch_fetch_grasp_bridge.srv import StretchSegmentation, StretchSegmentationRequest, StretchSegmentationResponse
from stretch_fetch_grasp_bridge.srv import StretchGraspPosev2, StretchGraspPosev2Request, StretchGraspPosev2Response

class TaskExecutorDummyNode(object):
    def __init__(self):
        # Create the clients
        rospy.loginfo("Waiting for service /stretch_segmentation/segment_objects")
        rospy.wait_for_service('/stretch_segmentation/segment_objects')
        self.get_segmentation = rospy.ServiceProxy('/stretch_segmentation/segment_objects', StretchSegmentation)
        rospy.loginfo("Waiting for service /stretch_grasp_pose_suggester")
        rospy.wait_for_service('/stretch_grasp_pose_suggester')
        self.get_grasp = rospy.ServiceProxy('/stretch_grasp_pose_suggester',StretchGraspPosev2)
        # self.segmentation_srv = rospy.Service('/stretch_segmentation/segment_objects', StretchSegmentation, self.service_callback)

    def run_task(self):
        # getting segmented cloud
        req = StretchSegmentationRequest()
        req.object_name = 'hello'
        print("sending segmentation request")
        resp = self.get_segmentation(req)
        print("got segmentation request")
        cloud = resp.segmented_point_cloud
        print(type(cloud))

        req = StretchGraspPosev2Request()
        req.point_cloud = cloud
        rospy.loginfo("sending grasp request!")
        grasp = self.get_grasp(req)
        print(type(grasp))
        print(grasp)
        print("got grasp")
        # getting grasp


def main():
    print("Task executor dummy node")
    node = TaskExecutorDummyNode()
    node.run_task()
if __name__ == '__main__':
    main()

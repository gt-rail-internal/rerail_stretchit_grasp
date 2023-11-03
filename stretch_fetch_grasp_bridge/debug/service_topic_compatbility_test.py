#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
class my_node:
    def __init__(self):
        rospy.init_node('my_node', anonymous=True)
        self.my_service = rospy.Service('/my_service', SetBool, self.my_service_callback)
        self.my_topic = rospy.Subscriber('/my_topic', String, self.my_topic_callback)
        self.last_string = None
    def my_service_callback(self,request):
        # Do something with the request
        self.last_string = None
        rospy.loginfo("Service called")
        response = SetBoolResponse()
        while(self.last_string is None):
            #spin rospy
            rospy.sleep(0.1)
        response.success = True
        response.message = self.last_string
        return response

    def my_topic_callback(self,msg):
        # Do something with the message
        self.last_string = msg.data
        rospy.loginfo("Message received: %s", msg.data)

if __name__ == '__main__':
    node = my_node()
    rospy.spin()

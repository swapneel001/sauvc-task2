#!/usr/bin/env python
""" Service Server and Publisher Code
"""
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool

class ServerPublisher:
    """Node Class containing constructor, service server, and publisher
      
        To use:
        >>> ServerPublsher()
        
        Attributes:
        service : Ros service server
        pub: ros publisher
        __msg: ros publish message

    """

    def __init__(self):
        self.service = rospy.Service("/thruster_state",
                                     SetBool, self.thrusterstate)
        self.pub = rospy.Publisher("/thruster_message", String, queue_size=10)
        self.__msg = String() #private datamember


    def thrusterstate(self, req):
        """ Service Server callback function, includes publisher.

        Arguments: service client request, i.e msg

        Returns: service server success state, success message,
        publisher message
        """
        if req.data:
            self.__msg.data = "ON"
            self.pub.publish(self.__msg)
            return True, "thruster is on"

        self.__msg.data = "OFF"
        self.pub.publish(self.__msg)
        return False, "thruster is off"

if __name__ == "__main__":
    """Main Function
    initialises the object of class ServerPublisher
    """
    
    rospy.init_node("task2pynodeOOP")
    ServerPublisher()
    RATE = rospy.Rate(10)
    RATE.sleep()
    rospy.spin()

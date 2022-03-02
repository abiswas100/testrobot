#!/usr/bin/env python
import rospy

from testrobots.msg import H_detection
from testrobots.msg import P_detection
        
def H_callbackdata(data):
    rospy.loginfo("In H_callback")
    rospy.loginfo(data.signal)  
    
    '''
    If signal is 0 take a set of actions -
        and 
    If signal is 1 take another set of action
    '''

def P_callbackdata(data):
    rospy.loginfo("In P_callback")
    rospy.loginfo(data.signal)    
    
    '''
    If signal is 0 take a set of actions -
        and 
    If signal is 1 take another set of action
    '''
                
def listener():
    '''
        Subscribe to message from the Detect Panel - Done
        Subscribe to message from the Detect Human - Done
        Subscribe to message from Specification Node about Safety
        Subscribe to Mapping Node
        Subscribe to Planning Node
        Subscribe to Odometer        
    '''
    rospy.init_node('Central_Decision', anonymous=True)
    rospy.loginfo("Hello I am Central Decision")
    
    rospy.Subscriber("/H_Detection_msg", H_detection, H_callbackdata)
    
    rospy.Subscriber("/P_Detection_msg", P_detection, P_callbackdata)
    
    rospy.spin()

    
if __name__ == "__main__":
    listener()
    
    
    
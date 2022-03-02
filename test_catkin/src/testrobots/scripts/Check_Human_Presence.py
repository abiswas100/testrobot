#!/usr/bin/env python

import rospy

from testrobots.msg import H_detection
from testrobots.msg import P_detection
        
def H_callbackdata(data):
    rospy.loginfo("In H_callback")
    rospy.loginfo(data.signal)  
    
    '''
    If signal is 0 take a set of actions git-
        and 
    If signal is 1 take another set of action
    '''
                
def listener():
    '''
        Subscribe to message from the Detect Human       
    '''
    rospy.init_node('Human_Check', anonymous = False)
    rospy.loginfo("Hello I am Human_Check")
    
    rospy.Subscriber("/H_Detection_msg", H_detection, H_callbackdata)

    rospy.spin()

    
if __name__ == "__main__":
    listener()
    
    
    
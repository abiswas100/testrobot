#!/usr/bin/env python

import rospy

from testrobots.msg import H_detection
from testrobots.msg import P_detection
from testrobots.msg import track
from testrobots.msg import stop
from testrobots.msg import tracking_updates


rospy.init_node('Specification_Check', anonymous = False)
stop_msg = rospy.Publisher("stop", stop)
        
def callbackdata(data):
    rospy.loginfo("In H_callback")
    rospy.loginfo(data.signal)  

def update_callback(t_data):
    rospy.loginfo("In tracking udpate callback")
    rospy.loginfo(t_data.update)  
        
    if t_data.update == 1: pass
        ## check for all distance comparisions       
        
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
        Subscribe to message from the Detect Human Tracking        
    '''
    
    rospy.loginfo("Hello I am Specification_Check")
    
    rospy.Subscriber("/P_Detection_msg", P_detection, P_callbackdata)
    rospy.Subscriber("/tack", track, callbackdata)
    rospy.Subscriber("/tupdate", tracking_updates, update_callback)
    
    rospy.spin()

    
if __name__ == "__main__":
    listener()
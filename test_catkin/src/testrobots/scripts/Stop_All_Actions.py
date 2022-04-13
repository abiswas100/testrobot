#!/usr/bin/env python

import rospy

from testrobots.msg import stop
        
def callback(data):
    rospy.loginfo("STOP movement")
    rospy.loginfo(data.movement)  
    
    '''
    If signal is 0 take a set of actions git-
        and 
    If signal is 1 take another set of action
    '''
                
def listener():
    rospy.init_node('All Stop', anonymous=False)
    
    rospy.Subscriber("stop", stop, callback)

    rospy.spin()

    
if __name__ == "__main__":
    listener()
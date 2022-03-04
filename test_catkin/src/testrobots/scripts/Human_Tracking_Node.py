#!/usr/bin/env python

import rospy

from testrobots.msg import track
from testrobots.msg import tracking_updates

rospy.init_node('Human_Tracking', anonymous=False)

tracking_update = rospy.Publisher("tupdate", tracking_updates)

        
def callbackdata(data):
    rospy.loginfo("In callback")
    rospy.loginfo(data.starttrack)  
    
    tu_msg = tracking_updates
    tu_msg.update = 1
    ## if human_found 
    if (data.starttrack == 0):
        rospy.loginfo(tu_msg)
        tracking_update.publish(tu_msg)
                    
                
def listener():
    '''
        Subscribe to message from the Detect Human Tracking symbol and camera      
    '''
    
    rospy.loginfo("Hello I am Human Tracking")
    
    rospy.Subscriber("track", track, callbackdata)

    rospy.spin()

    
if __name__ == "__main__":
    listener()
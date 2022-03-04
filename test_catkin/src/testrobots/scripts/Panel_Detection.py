#!/usr/bin/env python

import rospy

from testrobots.msg import track
from testrobots.msg import tracking_updates

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan 

rospy.init_node('Human_Tracking', anonymous=False)

tracking_update = rospy.Publisher("tupdate", tracking_updates)

def Image_callback(data):
    pass


def callbackdata(data):
    rospy.loginfo("In callback")
    rospy.loginfo(data.starttrack)  
    
    tu_msg = tracking_updates
    tu_msg.update = 1
    ## if human_found 
    if (data.starttrack == 0):
        rospy.loginfo(tu_msg)
        tracking_update.publish(tu_msg)
        
def Laser_callback():
    pass

def Image_callback():
    pass
                    
                
def listener():
    '''
        Subscribe to message from the Detect Human Tracking symbol and camera      
    '''
    
    rospy.loginfo("Hello I am Human Tracking")
    
    
    
    ### Depth Camera Input Subscribers
    rospy.Subscriber("/camera", Image, Image_callback)
    rospy.Subscriber("/scan", LaserScan, Laser_callback)
    rospy.spin()

    
if __name__ == "__main__":
    listener()
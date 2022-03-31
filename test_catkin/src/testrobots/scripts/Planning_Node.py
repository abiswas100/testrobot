#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from testrobots.msg import stop
from testrobots.msg import move
from testrobots.msg import tracking_updates
        
def callback(data):
    rospy.loginfo("STOP movement")
    rospy.loginfo(data.movement)  
    
    '''
    If signal is 0 take a set of actions git-
        and 
    If signal is 1 take another set of action
    '''
def Image_callback(data):
    pass

def move_callback(data):
    rospy.loginfo("STOP movement")
    rospy.loginfo(data.movement)  
    
    '''
    If signal is 0 take a set of actions git-
        and 
    If signal is 1 take another set of action
    '''
    
def Lasercallback(data):
    pass

def test_updatecallback(data):
    pass
    
                
def listener():
    rospy.init_node('Planner', anonymous=False)
    
    rospy.Subscriber("movement_msg", move,move_callback)
    rospy.Subscriber("/scan", LaserScan, Lasercallback)
    rospy.Subscriber("/camera", Image, Image_callback)
    rospy.Subscriber("stop", stop, callback)
    rospy.Subscriber("/tupdate", tracking_updates, test_updatecallback)

    rospy.spin()

    
if __name__ == "__main__":
    listener()
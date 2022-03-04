#!/usr/bin/env python

from multiprocessing.sharedctypes import Value
import rospy

import roslaunch
import threading
from threading import Thread

from testrobots.msg import H_detection
from testrobots.msg import P_detection
from testrobots.msg import move
from testrobots.msg import stop
from testrobots.msg import tracking_updates

rospy.init_node('Central_Decision', anonymous=True)
move_msg = rospy.Publisher("movement_msg", move)
# import asyncio

     
#         #launch another node 
# def start_action():
#     print("starting ..")
#     # r = rospy.Rate(20)
#     package = 'testrobots'
#     executable = 'Move_Towards_Goal.py'
#     node = roslaunch.core.Node(package, executable)
    
#     launch = roslaunch.scriptapi.ROSLaunch()
#     launch.start()
    
#     # t = Timer(5, ds)
#     task = launch.launch(node)
    
    
#     print (task.is_alive())

    
    
def H_callbackdata(data):
    rospy.loginfo("In H_callback")
    # rospy.loginfo("Signal is",data.signal,"so will start Move_to_Goal")  
    rospy.loginfo(data)
    
    msg = move()
    msg.movement = 1
    '''
    If signal is 0 take a set of actions -
        and 
    If signal is 1 take another set of action
    '''
    if (data.signal == 0): 
        # while not rospy.is_shutdown():  
        rospy.loginfo(msg)
        move_msg.publish(msg)
        

        
def P_callbackdata(data):
    rospy.loginfo("In P_callback")
    rospy.loginfo(data.signal)    
    
    '''
    If signal is 0 take a set of actions -
        and 
    If signal is 1 take another set of action
    '''

def stopper(data):
    rospy.loginfo(data)
                
def test_updatecallback(data):
    pass
                    
def listener():
    '''
        Subscribe to message from the Detect Panel - Done
        Subscribe to message from the Detect Human - Done
        Subscribe to message from Specification Node about Safety
        Subscribe to Mapping Node
        Subscribe to Planning Node
        Subscribe to Odometer        
    '''

    rospy.loginfo("Hello I am Central Decision")
    
    # start_action()
    rospy.Subscriber("/H_Detection_msg", H_detection, H_callbackdata)
    
    rospy.Subscriber("/P_Detection_msg", P_detection, P_callbackdata)
    
    rospy.Subscriber("/stop", stop, stopper)
    
    rospy.Subscriber("/tupdate", tracking_updates, test_updatecallback)

    rospy.spin()

    
if __name__ == "__main__":
    
    try:
        listener()
    except rospy.ROSInterruptException: pass
    
    
    
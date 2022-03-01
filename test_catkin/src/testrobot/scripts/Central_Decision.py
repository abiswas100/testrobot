#!/usr/bin/env python
import rospy

from robot.msg import H_detection
        
def callback(msg):
    rospy.loginfo("In callback")
    rospy.loginfo("Human Flag is - ", msg.signal)    
                
def main():
    
    '''
        Subscribe to message from the Detect Panel
        Subscribe to message from the Detect Human - Done
        Subscribe to message from Specification Node about Safety
        Subscribe to Mapping Node
        Subscribe to Planning Node
        Subscribe to Odometer        
    '''
    rospy.init_node('Central_Decision', anonymous = True)
    rospy.loginfo("Hello I am Central Decision")
    
    rospy.Subscriber("H_Detection_msg", H_detection, callback)
    

    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass
    
    rospy.spin()
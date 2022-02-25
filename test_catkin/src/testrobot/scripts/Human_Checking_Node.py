#!/usr/bin/env python
import rospy
from std_msgs.msg import String

hello_str = "I decide on human presence"
rospy.init_node('Human_Checking', anonymous = False)
pub = rospy.Publisher('Human_Checking', String, queue_size=10)
pub.publish(hello_str)
    
'''
Publish a message to start Human Motion Tracking
Publish a message to start Specification Decision Node
Publish a message to Central Decision Node 
''' 
def callback(msg: String):
    # pub.publish(hello_str)
    pass
    
            
def main():
    
    try:
        pass
        # rospy.Subscriber("/mapping", String, callback, queue_size=10)
        '''
        Subscribe to Detect Human
        '''
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
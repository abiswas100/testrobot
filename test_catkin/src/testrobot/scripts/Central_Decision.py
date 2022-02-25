#!/usr/bin/env python
import rospy
from std_msgs.msg import String

hello_str = "I am the Central Decision"
rospy.init_node('Central_Decision', anonymous = False)
pub = rospy.Publisher('Central_Decision', String, queue_size=10)
pub.publish(hello_str)

'''
Publication List

'''
        
def callback(msg: String):
    # pub.publish(hello_str)
    pass
    
            
def main():
    
    try:
        pass
        # rospy.Subscriber("", String, callback, queue_size=10)
        '''
        Subscribe to message from the Detect Panel
        Subscribe to message from the Detect Human
        Subscribe to message from Specification Node about Safety
        Subscribe to Mapping Node
        Subscribe to Planning Node
        Subscribe to Odometer        
        '''
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
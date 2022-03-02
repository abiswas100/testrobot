#!/usr/bin/env python
import rospy
from std_msgs.msg import String

hello_str = "Critical Action - ALL STOP"
rospy.init_node('ALLStop', anonymous = False)
pub = rospy.Publisher('All_Stop', String, queue_size=10)
pub.publish(hello_str)
    
        
def callback(msg: String):
    # pub.publish(hello_str)
    pass
    
            
def main():
    
    try:
        pass
        # rospy.Subscriber("/mapping", String, callback, queue_size=10)
        '''
        Subscribe to message from the Central Decision Node
        '''
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
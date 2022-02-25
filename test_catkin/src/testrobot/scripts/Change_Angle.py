#!/usr/bin/env python
import rospy
from std_msgs.msg import String

hello_str = "I change angle by 10 degrees"
rospy.init_node('Change_Angle', anonymous = False)
pub = rospy.Publisher('Change_Angle', String, queue_size=10)
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
        Subscribe to Odometer
        Subscribe to Mapping and Planning if required
        '''
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
    


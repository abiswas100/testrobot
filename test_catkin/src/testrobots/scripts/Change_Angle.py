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
    rospy.loginfo("hello in Change Angle")
    '''
        Subscribe to message from the Central Decision Node
        Subscribe to Odometer
        Subscribe to Mapping and Planning if required
    '''
    try:
        rospy.spin()
        pass
        # rospy.Subscriber("/mapping", String, callback, queue_size=10)
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
    


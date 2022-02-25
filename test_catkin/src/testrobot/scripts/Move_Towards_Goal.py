#!/usr/bin/env python
import rospy
from std_msgs.msg import String

hello_str = "I move the robot towards goal"
rospy.init_node('Move_Goal', anonymous = False)
pub = rospy.Publisher('Moving_to_Goal', String, queue_size=10)
pub.publish(hello_str)
    
        
def callback(msg: String):
    # pub.publish(hello_str)
    pass
    
            
def main():
    
    try:
        pass
        # rospy.Subscriber("", String, callback, queue_size=10)
        '''
        Subscribe to message from the Central Decision Node
        Subscribe to Odometer
        # Subscribe to Mapping and Planning if required
        Subscribe to Message from Detect Panel Stack
        Subscribe to Lidar
        '''
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
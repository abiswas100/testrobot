#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

hello_str = "I do planning for Robot"
rospy.init_node('Mapping_Node', anonymous = False)
pub = rospy.Publisher('Map', String, queue_size=10)
pub.publish(hello_str)
    
        
def main():
#     '''Will publish when Laser is available'''
#     # pub.publish(hello_str)
    pass
    
      
def main():
    rospy.loginfo("I am mapping node")
    rospy.spin()    
    try:
        pass
        # rospy.Subscriber("/scan", LaserScan, callback, queue_size=10)
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
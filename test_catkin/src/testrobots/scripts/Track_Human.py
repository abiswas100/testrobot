#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge



hello_str = "I Track Humans"
rospy.init_node('Panel_Detection', anonymous = False)
pub_Image = rospy.Publisher('Panel', Image, queue_size=10)
pub = rospy.Publisher('Panel_Detection_', String, queue_size=10)
bridge = CvBridge()  


        
def main():
    
    ### Depth Camera Input Subscribers
    
    # rospy.Subscriber("/camera", Image, image_callback, queue_size=10)
    
    # rospy.Subscriber()
    try:
        rospy.Subscriber()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    
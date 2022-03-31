#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
# from sensor_msgs.msg import Image

# from testrobots.msg import stop
# from testrobots.msg import move
# from testrobots.msg import tracking_updates
        
# def Image_callback(data):
#     pass

def Lasercallback(data):
    pass

                
def listener():
    rospy.init_node('Planner', anonymous=False)
    
    rospy.Subscriber("/scan", LaserScan, Lasercallback)
    # rospy.Subscriber("/camera", Image, Image_callback)
    
    rospy.spin()

    
if __name__ == "__main__":
    listener()
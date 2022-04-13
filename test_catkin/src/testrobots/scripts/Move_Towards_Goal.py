#!/usr/bin/env python
import rospy
#!/usr/bin/env python

import rospy

from testrobots.msg import H_detection
from testrobots.msg import P_detection
from testrobots.msg import move

def callbackdata(data):
    rospy.loginfo("Will publish movement action")
    rospy.loginfo(data.movement)  
    
    '''
    If signal is 0 take a set of actions git-
        and 
    If signal is 1 take another set of action
    '''
                
def listener():
    '''
        Subscribe to message from the Detect Human       
    '''
    rospy.init_node('Move_Goal', anonymous = False)
    rospy.loginfo("Hello I am Move to Goal")
    
    rospy.Subscriber("/movement_msg", move, callbackdata)

    rospy.spin()

    
if __name__ == "__main__":
    listener()
    
    
    
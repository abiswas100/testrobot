#!/usr/bin/env python
#!/usr/bin/env python3

import rospy

from testrobots.msg import H_detection
from testrobots.msg import track

rospy.init_node('Human_Check', anonymous=False)
track_msg = rospy.Publisher("track", track)
        
def H_callbackdata(data):
    rospy.loginfo("In H_callback")
    rospy.loginfo(data.signal)  
    
    msg_t = track()
    msg_t.starttrack = 1
    
    '''
    If signal is 0 take a set of actions git-
        and 
    If signal is 1 take another set of action
    '''
    if (data.signal == 0): 
 
        rospy.loginfo(msg_t)
        track_msg.publish(msg_t)
    
                
def listener():
    '''
        Subscribe to message from the Detect Human       
    '''
    
    rospy.loginfo("Hello I am Human_Check")
    
    rospy.Subscriber("/H_Detection_msg", H_detection, H_callbackdata)

    rospy.spin()

    
if __name__ == "__main__":
    listener()
#!/usr/bin/env python
#!/usr/bin/env python3



import rospy
from testrobots.msg import move

def callbackdata(data):
    rospy.loginfo("Will change the angle")
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
    rospy.init_node('Change Angle', anonymous = False)
    
    
    rospy.Subscriber("/movement_msg", move, callbackdata)

    rospy.spin()

    
if __name__ == "__main__":
    listener()
    
    
    
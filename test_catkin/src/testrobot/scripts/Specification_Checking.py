import rospy
from std_msgs.msg import String

hello_str = "I check if specifications are running or not"
rospy.init_node('Specification_Node', anonymous = False)
pub = rospy.Publisher('Specification', String, queue_size=10)
pub.publish(hello_str)
    
        
def callback(msg: String):
    # pub.publish(hello_str)
    pass
    
            
def main():
    
    try:
        pass
        # rospy.Subscriber("/mapping", String, callback, queue_size=10)
        '''
        Subscribe to message from Human_Checking_Node to start this node
        Subsrcibe to the Human Tracking Node
        Subscribe to the Lidar
         '''
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
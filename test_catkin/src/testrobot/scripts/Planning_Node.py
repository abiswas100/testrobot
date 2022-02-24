import rospy
from std_msgs.msg import String

hello_str = "I do planning for Robot"
rospy.init_node('Planner', anonymous = False)
pub = rospy.Publisher('Planning_Node', String, queue_size=10)
pub.publish(hello_str)
    
        
def callback(msg: String):
    pub.publish(hello_str)
    pass
    
            
def main():
    
    try:
        pass
        # rospy.Subscriber("/mapping", String, callback, queue_size=10)
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
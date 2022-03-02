#!/usr/bin/env python

import rospy
# from sensor_msgs.msg import Image
from std_msgs.msg import String
from testrobots.msg import P_detection

# import yolo as Yolo

        
def talker():
    msg_pub = rospy.Publisher("P_Detection_msg", P_detection)
    
    rospy.init_node('Panel_Detection', anonymous = True)

    r = rospy.Rate(10)
    
    msg = P_detection()
    msg.signal = 1 
    
    # human_flag = 1
    
    # if human_flag == 1:
    #     # rospy.loginfo_once("Human Detected on Camera")

    while not rospy.is_shutdown():        
        # rospy.loginfo(msg)
        msg_pub.publish(msg)
        rospy.loginfo("Human Detected on Camera")
        r.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException: pass
    rospy.spin()
    
    
    
    
    
    
    
    
    

# def image_processing_publish(cv_img):       
#     yolo_output = Yolo.Yolo_imp(cv_img)
#     print("for left")
#     left_output = bridge.cv2_to_imgmsg(yolo_output)
#     while not rospy.is_shutdown():
#         pub_Image.publish(left_output)
#         '''will add this here when real implementation happens'''
#         # pub.publish(hello_str)
#         rospy.spin()   
        
# def image_callback(msg: Image):
#     cv_img =  bridge.imgmsg_to_cv2(msg)
#     image_processing_publish(cv_img)


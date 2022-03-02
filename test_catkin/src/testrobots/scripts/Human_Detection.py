#!/usr/bin/env python

import rospy


from testrobots.msg import H_detection
    
def talker():

    msg_pub = rospy.Publisher("H_Detection_msg", H_detection)
    
    rospy.init_node('Human_Detection', anonymous=True)

    r = rospy.Rate(10)
    
    msg = H_detection()
    msg.signal = 1 
    
    # human_flag = 1
    
    # if human_flag == 1:
    #     # rospy.loginfo_once("Human Detected on Camera")

    while not rospy.is_shutdown():        
        # rospy.loginfo(msg)
        msg_pub.publish(msg)
        rospy.loginfo("Human Detected on Camera")
        r.sleep()
        
    
    ### Depth Camera Input Subscribers
    # rospt.Subscriber("/camera", Image, image_callback)
    
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException: pass
    rospy.spin()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    




# bridge = CvBridge()  

        
# def yolo_processing(cv_img):       
#     ''' yolo processing node computes detection 
#         and returns new image with detection and 
#         human_flag which turns true if the human is detected
#     '''
#     yolo_output = Yolo.Yolo_imp(cv_img)
#     output = bridge.cv2_to_imgmsg(yolo_output)
#     human_flag = 0
#     '''
#     Add a custom msg called Human Detected -
#     It is published if a human is detected 
#     '''

#     while not rospy.is_shutdown():
#         pub_Image.publish(output)
#         # if human_flag is 1:
#         #     rospy.loginfo_once("Human Detected on Camera")
#         #     msg.signal = 1 
#         #     msg_pub.publish(msg)

        
# def image_callback(msg: Image):
#     cv_img =  bridge.imgmsg_to_cv2(msg)
#     yolo_processing(cv_img)


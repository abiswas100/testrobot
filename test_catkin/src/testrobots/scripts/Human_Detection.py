#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan 
import csv
from cv_bridge import CvBridge
import yolo as Yolo
from testrobots.msg import H_detection  
bridge = CvBridge()  


pub = rospy.Publisher("H_Detection_image", Image, queue_size=10)    
msg_pub = rospy.Publisher("H_Detection_msg", H_detection, queue_size=1)

def yolo_processing(cv_img):       
    ''' yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
    '''
    msg = H_detection()
    msg.signal = -1
    yolo_output, object_label= Yolo.Yolo_imp(cv_img)
    output = bridge.cv2_to_imgmsg(yolo_output)
    
    '''
    Add a custom msg called Human Detected -
    It is published if a human is detected 
    '''
    if(object_label == 'person'):
        rospy.logwarn("Human Detected on Camera")
        msg.signal = 1 
        
    rospy.logwarn(msg.signal)
    
    #publish the message and the image
    msg_pub.publish(msg)
    pub.publish(output)
    
        
def image_callback(data):
    # print("here in callbaack")
    cv_img =  bridge.imgmsg_to_cv2(data)
    yolo_processing(cv_img)

def DepthCamSub(data):
    depth_cv_img =  bridge.imgmsg_to_cv2(data)
    


def main():
    rospy.init_node('Human_Detection', anonymous=False)
    
    ### Depth Camera Input Subscribers
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback,queue_size=10)
    rospy.Subscriber("/camera/depth/image_raw", Image, DepthCamSub)
    while not rospy.is_shutdown():
        rospy.spin()
if __name__ == "__main__":
    # try:
    #     talker()
    # except rospy.ROSInterruptException: pass

    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    




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

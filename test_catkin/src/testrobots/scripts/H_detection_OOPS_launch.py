#!/usr/bin/env python
#!/usr/bin/env python3


from numpy import NaN
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2 as pc2
import csv
import cv2
from cv_bridge import CvBridge
# import yolo as Yolo
import numpy as np
import os
import time

from testrobots.msg import H_detection
from testrobots.msg import stop  

bridge = CvBridge() 
class Detection(object):

    def __init__(self):
        self.queue = []
        self.queue.append([0,0])        
        self.central_pixel = []
       
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback,queue_size=1)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.DepthCamSub, queue_size=1)
        # rospy.Subscriber("/camera/depth/points",pc2, Depthcloud, queue_size=1)

        # publishing topics
        self.pub = rospy.Publisher("H_Detection_image", Image, queue_size=1)    
        self.msg_pub = rospy.Publisher("H_Detection_msg", H_detection, queue_size=1)
        self.stop_msg =  rospy.Publisher("Stop_msg", stop, queue_size=1)
        self.vector_pub = rospy.Publisher("H_Vector", Image, queue_size=1)
        
        #initialize csv file
        self.path = os.getcwd()
        self.csvpath = self.path+"/human_motion.csv"
        # open the file in the write mode
        
        header = ['center_x', 'center_y', 'Distance']
        self.csv_file = open(self.csvpath, 'w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(header)
        
    def image_callback(self,data):
        # print("here in callbaack")
        cv_img =  bridge.imgmsg_to_cv2(data)
        
        tracking_img = cv_img

        self.yolo_processing(cv_img)
        self.human_motion_tracking(tracking_img)

    def yolo_processing(self,cv_img):       
        ''' 
        yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
        '''
        msg = H_detection()
        msg.signal = -1

        
        yolo_output, object_label, center_pixels = self.Yolo_imp(cv_img)
        
       # checking center_pixels and setting center_pixel to 0         
        if len(center_pixels) == 0: 
            self.center_pixel = []
            print("no center pixel in yolo_processing...",self.center_pixel,"length of center pixel", len(self.center_pixel))
        else:
            self.center_pixel = center_pixels[0]
            print("center_pixel in yolo processing- ", self.center_pixel, "length of center pixel", len(self.center_pixel))

        #making the yolo output into a ros image version        
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
        self.msg_pub.publish(msg)
        self.pub.publish(output)
        
        # checking if center_pixels is empty and then setting the past center
        if len(self.center_pixel) == 0: pass
        else:
            self.queue.append(self.center_pixel)



    def human_motion_tracking(self, tracking_img):
        
        if len(self.center_pixel) == 0: 
            output = bridge.cv2_to_imgmsg(tracking_img)
            self.vector_pub.publish(output)

        else:
            print("current center pixel in human",self.center_pixel)    
            center_x = self.center_pixel[1]
            center_y = self.center_pixel[0]

            # check if stack is empty 
            if len(self.queue) == 0: pass
            else: 
                past = self.queue.pop(0)
                print("past value", past)

            past_center_x = past[1]
            past_center_y = past[0]
            
            #draw the arrow on the image            
            start_point = (center_x,center_y)
        
            end_point = (past_center_x, past_center_y)
            #red color in BGR
            color = (0, 255 , 0)
            
            thickness = 20
            
            image = cv2.arrowedLine(tracking_img, start_point, end_point,
                                        color, thickness)
            
            output = bridge.cv2_to_imgmsg(image)
            self.vector_pub.publish(output)
        


    def DepthCamSub(self,depth_data):
        depth_cv_img =  bridge.imgmsg_to_cv2(depth_data)

        if len(self.center_pixel) == 0:
            print("no centers in depth")
            rospy.sleep(0.5)
            
        else:
            print("center_pixel in depth- ", self.center_pixel, "length of center pixel", len(self.center_pixel))
            # print("center pixel in depthcam",self.center_pixel)
            center_x = self.center_pixel[1]
            center_y = self.center_pixel[0]

            depth = depth_cv_img[self.center_pixel[1]][self.center_pixel[0]]
            
            msg = stop()
            msg.stop = -1            
            
            #changing Nan Values to 0
            if depth == "nan":
                depth = 0
            
            
            data_to_write = [center_x,center_y,depth]

            self.writer.writerow(data_to_write)
            

            print("distance of human in depthcam - ", depth_cv_img[self.center_pixel[1]][self.center_pixel[0]])
            
            if depth <= 1.5 : 
                rospy.logfatal("Human too close ... Stop Immediately")
                msg.stop = 1
                rospy.logwarn(msg.stop)            
            
            self.stop_msg.publish(msg)            
            print("stop signal value", msg.stop)
            rospy.sleep(0.5)
                

    def Yolo_imp(self,img_data): 
        start_time = time.perf_counter()
        
        #switching directories from .ros to test..scripts file
        current_path = os.getcwd()
        home_path = current_path[:-4]
        scripts_path = home_path + "testrobot/test_catkin/src/testrobots/scripts"
        
        
        if current_path == '/home/apramani/testrobot/test_catkin/src/testrobots/scripts':
            pass
        else:
            os.chdir(scripts_path)
        
        # 
        cfg = scripts_path+"/yolov3.cfg"
        weights = scripts_path + "/yolov3.weights"
        
        net = cv2.dnn.readNet(cfg,weights)
        # net = cv2.dnn.readNet('yolov3.cfg', 'yolov3.weights')
        classes = []

        with open('coco.names', 'r') as f:
            
            classes = f.read().splitlines()

        height,width,_ = img_data.shape
        
        blob = cv2.dnn.blobFromImage(img_data, 1/255, (256, 256), (0,0,0), swapRB=False, crop=False)

        net.setInput(blob)

        output_layers_names = net.getUnconnectedOutLayersNames()

        layerOutputs = net.forward(output_layers_names)

        boxes = []
        confidences = []
        class_ids = []
        center_pixels = []
        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
                
                '''
                    scale the bounding box coordinates back relative to the
                    size of the image, keeping in mind that YOLO actually
                    returns the center (x, y)-coordinates of the bounding
                    box followed by the boxes' width and height
                '''

                if confidence > 0.5:                                
                    center_x = int(detection[0]*width)
                    center_y = int(detection[1]*height)
                    w = int(detection[2]*width)
                    h = int(detection[3]*height)
                    
                    '''
                        use the center (x, y)-coordinates to derive the top and left corner of the bounding box
                    '''

                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    
                    boxes.append([x,y,w,h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    
                    if class_id == 0:                   # 0 belongs to label person
                        center_pixels.append([center_x,center_y])

        print("---------------------------------------------------")
        print("center pixels in yolo",center_pixels)
        
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        
        object_label = ""
        
        font = cv2.FONT_HERSHEY_PLAIN
        colors = np.random.uniform(0, 255, size=(len(boxes), 3))
        if len(indexes)>0:

            for i in indexes.flatten():
                x,y,w,h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = str(round(confidences[i], 2))
                area = 0
                print("")
                
                print("label -",label,
                ", confidence", confidence,
                ", area of Bounding Box  - ",w*h)

                color = colors[i]
                cv2.rectangle(img_data,(x,y), (x+w, y+h), color, 2)
                cv2.putText(img_data, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)
            
        try:
            object_label = label
        except UnboundLocalError: 
            print("no label formed in YOLO")

        # object_label = "person"
        end_time = time.perf_counter ()
        print("")
        
        # cv2.imwrite('yolo_img.jpeg', img_data)

        #changing directory to .ROS folder
        os.chdir(current_path)
        
        return img_data, object_label, center_pixels 


def main():
    rospy.init_node('Human_Detection', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
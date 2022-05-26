#!/usr/bin/env python3
import os

import rospy
try:
    import cv2
except ImportError:
    import sys
    ros_path = '/opt/ros/neotic/lib/python2.7/dist-packages'
    sys.path.remove(ros_path)
    import cv2
    sys.path.append(ros_path)
    
from os import chdir
import numpy as np
import time
import csv


def Yolo_imp(img_data): 
    start_time = time.perf_counter ()
    corners = []
    net = cv2.dnn.readNet('yolov3.cfg','yolov3.weights')
    classes = []

    with open('coco.names', 'r') as f:
        
        classes = f.read().splitlines()

    # img_name = name1 = input("Enter name of the image file: ")
    # img_data  = cv2.imread('image.jpeg')
    # print(img_data)
    height,width,_ = img_data.shape
    # print(height,width)
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
                # print(class_id)
                # if class_id == 0:
                    # center_pixels.append([center_x,center_y])
    print("---------------------------------------------------")
    # print("center pixels in yolo",center_pixels)

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
            
            leftbottom_corner = [x,y]
            rightbottom_corner = [x+w,y]
            lefttop_corner = [x,y+h]
            righttop_corner =  [x+w,y+h]
            '''
                Add code to find the center from the coordinates
            '''    
            # center from cordinates: by Apala
            # center__x = int(x + w/2) # x coordinate of left bottom plus half of width
            # center__y = int(y + h/2) # y coordinate of left bottom plus half of height
            # center_bb = (center__x, center__y) #center of bounding box
            # print("apala ka center",center_bb)
            probable_center = (center_x, center_y)
            center_pixels.append([center_x,center_y])
            print("center pixels in yolo",center_pixels)
            # center_pixels.append([center__x,center__y])
            corners.append([leftbottom_corner,rightbottom_corner,lefttop_corner,righttop_corner])
             
            print("corners in yolo", leftbottom_corner,rightbottom_corner,lefttop_corner,righttop_corner)
            
            cv2.rectangle(img_data,(x,y), (x+w, y+h), color, 2)
            cv2.putText(img_data, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)

    try:
        object_label = label
    except UnboundLocalError: 
        print("no label")

    # object_label = "person"
    end_time = time.perf_counter ()
    print("")
    # print(end_time - start_time, "seconds")
    cv2.imwrite('yolo_img.jpeg', img_data)
    return img_data, object_label, center_pixels,  corners, confidence 
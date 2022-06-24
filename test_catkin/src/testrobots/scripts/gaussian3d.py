#!/usr/bin/env python
#!/usr/bin/env python3

from array import array
from cmath import sqrt
from os import device_encoding
from numpy import NaN, cov
import rospy
import ros_numpy
import matplotlib.pyplot as plt
import seaborn as sns

from numpy import random

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from testrobots.msg import Meannn
from testrobots.msg import Deviation



from geometry_msgs.msg import Point, PointStamped

import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError
import yolo as Yolo
import numpy as np
import time

bridge = CvBridge() 
class Detection(object):

    def __init__(self):
        
        self.mean = Point()
        self.cov_mat = []
        self.std_dev = Point()
       
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=1)
        
                
        # publishing topics
        self.publish_mean = rospy.Publisher("mean", Meannn, queue_size=1)   
        self.publish_std_dev = rospy.Publisher("deviation", Deviation, queue_size=1)
        # self.publish_covar = rospy.Publisher("covar", pc2, queue_size=1) # Covariance matrix needs to be published as a covariance array

    def cloud_callback(self,data):
        print("Here in Pointcloud callback.........................................")
        
        
        # creating the mean and covariance messages to publish
        mean = Meannn()
        mean_x = Meannn()
        mean_y = Meannn()
        mean_z = Meannn()
        std_dev = Deviation()
        dev_x = Deviation()
        dev_y = Deviation()
        dev_z = Deviation()
        
        
        pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False) 
        
        arr = np.array(pcl_np)
        mean.mean_value = np.mean(arr)
        blahh = arr.mean(axis=1)
        # mean_x.mean_value = blahh[0]
        # mean_y.mean_value = blahh[1]
        # mean_z.mean_value = blahh[2]
        
        # print(blahh)
        print("mean size:", len(blahh))
        
        # print("mean_x: ", mean_x.mean_value, "mean_y: " , mean_y.mean_value, "mean_z: ", mean_z.mean_value)
        
        # print(arr)
        print("size of original array is: " , len(arr),"X",len(arr[0]))
        
        cov_mat = np.cov(arr)
        
        std_dev.std_dev_value = np.std(arr)
        duhh = arr.std(axis=0)
        dev_x.std_dev_value = duhh[0]
        dev_y.std_dev_value = duhh[1]
        dev_z.std_dev_value = duhh[2]
        
        print(duhh)
        
        # print("dev_x: ",dev_x.std_dev_value, "dev_y: " , dev_y.std_dev_value, "dev_z: ", dev_z.std_dev_value)
        

        # print("covariance matrix is ::", cov_mat)
       
        print("size of cov_mat is: ", len(cov_mat),"X",len(cov_mat[0])) 
        print("std deviation is: ", std_dev.std_dev_value)          
        print("mean is: ", mean.mean_value)
        # print()
        
        x,y = np.random.multivariate_normal(blahh,cov_mat,3)
        plt.plot(x,y,'x')
        plt.axis('equal')
        plt.show()
        
        
        # s = np.random.normal(mean.mean_value,std_dev.std_dev_value,1000)
        # sns.distplot(random.normal(size=1000), hist=False)
        # plt.show()
        # plt.close()
        
        
        
        
        self.publish_mean.publish(mean)
        self.publish_std_dev.publish(std_dev)
        # self.publish_covar.publish(cov_mat)   




def main():
    rospy.init_node('Compute_Gaussian', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
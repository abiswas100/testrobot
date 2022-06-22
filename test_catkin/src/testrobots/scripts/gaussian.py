#!/usr/bin/env python
#!/usr/bin/env python3

from cmath import sqrt
from numpy import NaN
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
       
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=1)
       
        
        # publishing topics
        self.publish_mean = rospy.Publisher("mean", pc2, queue_size=1)   
        self.publish_covar = rospy.Publisher("covar", pc2, queue_size=1)

    def cloud_callback(self,data):
        print("Here in Pointcloud callback")
        header = data.header
        fields = [PointField('x',0, PointField.FLOAT32, 1),
                  PointField('y',4, PointField.FLOAT32, 1),
                  PointField('z',8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]
        pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=True) 
        arr = np.array(pcl_np)
        mean = np.mean(arr)
        cov_mat=np.cov(arr)
        # mean = np.mean(pcl_np)
        # cov_mat = np.cov(pcl_np)
        self.publish_mean.publish(mean)
        self.publish_covar.publish(cov_mat)   
        x = random.normal(loc=mean, scale=sqrt(cov_mat), size=(3, 3))
        sns.distplot(random.normal(size=1000), hist=False)
        plt.show()




def main():
    rospy.init_node('Depth_Image_Extract', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
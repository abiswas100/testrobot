#!/usr/bin/env python
#!/usr/bin/env python3

#imports 
import sys
import rospy
from array import array
from cmath import sqrt
from os import device_encoding
# from cv2 import HOUGH_MULTI_SCALE
from numpy import NaN, cov
# import rospy
import ros_numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import seaborn as sns
from sklearn.cluster import DBSCAN
from numpy import random
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Polygon, PolygonStamped
from visualization_msgs.msg import Marker, MarkerArray
from testrobots.msg import Meannn
from testrobots.msg import Deviation
from testrobots.msg import Velocity
from kf import KF
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import norm
from sympy import Symbol, Matrix
from sympy.interactive import printing
from geometry_msgs.msg import Point, PointStamped, Point32
import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError
import yolo as Yolo
import numpy as np
import time
import math

class Detection(object):

    def __init__(self):
        
        self.mean = Point()
        self.cov_mat = []
        self.std_dev = Point()
        self.pos_mean_queue = []
        self.pointcloud_queue = []
        t_now = rospy.get_time()
        print(t_now)
        pos_mean = [0.00, 0.00,t_now]
        self.pos_mean_queue.append(pos_mean)
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=1)
        
                
        # publishing topics
        self.pub_mean_x = rospy.Publisher("mean_x", Meannn, queue_size=1)   
        self.pub_mean_y = rospy.Publisher("mean_y", Meannn, queue_size=1)  
        self.pub_mean_z = rospy.Publisher("mean_z", Meannn, queue_size=1)  
        
        self.pub_vel_x = rospy.Publisher("vel_x",Velocity,queue_size=1)
        self.pub_vel_y = rospy.Publisher("vel_y",Velocity,queue_size=1)
        self.pub_vel_z = rospy.Publisher("vel_z",Velocity,queue_size=1)
        
        self.publish_std_dev = rospy.Publisher("deviation", Deviation, queue_size=1)
        
        
        self.human_polygon = rospy.Publisher("Human_polygon", PolygonStamped, queue_size=1)
        self.human_marker = rospy.Publisher("Human_marker", Marker, queue_size=2)
        self.human_pred = rospy.Publisher("Human_prediction", MarkerArray, queue_size=2)
    
    
    def cloud_callback(self,data):
        print("Here in Pointcloud callback.........................................")
        header = data.header
        
        # creating the mean and covariance messages to publish 
        mean = Meannn()
        mean_x = Meannn()
        mean_y = Meannn()
        mean_z = Meannn()
        std_dev = Deviation()
        dev_x = Deviation()
        dev_y = Deviation()
        dev_z = Deviation()
        vx = Velocity()
        vy = Velocity()
        vz = Velocity()
        
       
        Human_Marker_cube = Marker()
        Human_prediction = MarkerArray()
        
        
        pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False) 
        
    
        #create a 2D array out of this pcl_np without the y values which are 0 after projection
        xzarray = []
        height, width = pcl_np.shape
        x_values = []
        z_values = []
        for i in range(0,height):
                point = pcl_np[i]
                x = point[0]
                z = point[2]
                x_values.append(x)
                z_values.append(z)
                value = [x,z]
                xzarray.append(value)
        
        x_np_array = np.array(x_values)
        z_np_array = np.array(z_values)

        mean_x = x_np_array.mean(axis=0)
        mean_z = z_np_array.mean(axis=0)
    
       
        
        mean_x_array = abs(x_np_array.mean(axis=0))
        mean_z_array = z_np_array.mean(axis=0)        
        cov_x = np.cov(x_np_array)
        cov_z = np.cov(z_np_array)
        
        # print("mean x", mean_x_array, "", "mean_z", mean_z_array)
        # print("Covariance x", cov_x, "", "Covariance_z", cov_z)
        
        
        # find mean for the XZ array
        xz_np_array = np.array(xzarray)
        mean2D = xz_np_array.mean(axis=1)   # axis  = 0 is for column, axis 1 for row and abs is to keep everything positive
        cov_xz = np.cov(xz_np_array)
        # cov_xz = np.cov(x_np_array,z_np_array)
        
        # Plot Gaussian 
        '''    Uncomment lines below to see gaussian plots
        '''
        
        # Y,Z= np.random.multivariate_normal(mean2D,cov_xz, 2)
        # plt.plot(Z)
        # plt.xlabel('Density')
        # plt.draw()
        # plt.pause(10)
        
        
        # compute DBSCAN - change eps and min_samples as required, eps- min distance between points
        # learn more from - https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
        DBSCAN_cluster = DBSCAN(eps=0.5, min_samples=30).fit(xz_np_array)
        labels = DBSCAN_cluster.labels_
        components = DBSCAN_cluster.components_ #copy of each core sample found by training
        feature = DBSCAN_cluster.n_features_in_ #number of features seen during fit
        
        # useful cluster has human with label 0, might change if more humans are added, x and y are points needed for plotting        useful_cluster = []
        useful_cluster = []
        x = []
        z = []
        for i in range(len(components)):
            if labels[i] == 0 :
                useful_cluster.append(components[i])
                point = components[i]
                x.append(point[0])
                z.append(point[1])
                
        

        
        if np.mean(x) == NaN or np.mean(z) == NaN:
            print("no human in view")
            pass
        else:
            
            #publish mean 
            # mean_y = 0.0
            # self.pub_mean_x.publish(np.mean(x))
            # self.pub_mean_y.publish(mean_y)
            # self.pub_mean_z.publish(np.mean(z))
            
            meanx = np.mean(x)
            meanz = np.mean(z)
            meany = 0.0
            t_now = rospy.get_time()
            pos_mean_now = [meanx, meanz,t_now]
            self.pos_mean_queue.append(pos_mean_now)
            
            Human_Marker_cube.header.frame_id = "camera_rgb_optical_frame"
            Human_Marker_cube.header.stamp = rospy.Time.now()
            Human_Marker_cube.ns = "basic_shapes"
            Human_Marker_cube.id = 1
            Human_Marker_cube.type = 1
            Human_Marker_cube.pose.position.x =  meanx
            Human_Marker_cube.pose.position.y = 0.0
            Human_Marker_cube.pose.position.z = meanz
            Human_Marker_cube.pose.orientation.x = 1.0
            Human_Marker_cube.pose.orientation.y = 1.0
            Human_Marker_cube.pose.orientation.z = 0.0
            Human_Marker_cube.pose.orientation.w = 0.0
            Human_Marker_cube.scale.x = 0.8
            Human_Marker_cube.scale.y = 0.8
            Human_Marker_cube.scale.z = 0.8
            Human_Marker_cube.color.a = 1.0
            Human_Marker_cube.color.r = 1.0
            Human_Marker_cube.color.g = 0.0
            Human_Marker_cube.color.b = 0.0
            
            
            # while not rospy.is_shutdown():
            self.human_marker.publish(Human_Marker_cube)
            
            #calculate velocity x,y,z
        
            '''
                find the velocity in x and z plane
            '''
            pos_mean_last = self.pos_mean_queue.pop(0)
            meanx_last, meanz_last,t_last = pos_mean_last[0],pos_mean_last[1],pos_mean_last[2]
            # print("Past meanx, meanz and time",meanx_last, meanz_last,t_last)
            
            # print("popped value",self.pos_mean_queue.pop(0))
            print("")
            dist_x, dist_z, time_diff = round(math.dist([meanx],[meanx_last]),2) , round(math.dist([meanz],[meanz_last]),2) , t_now - t_last  #time in seconds
            vx, vz, vy = round((dist_x/time_diff),2), round((dist_z/time_diff),2), 0.0  # speed in m/sec
            
            acc_x, acc_z = round((vx/time_diff),2), round((vz/time_diff),2)
            print("accleration in x and z", acc_x, acc_z)
            acc_y = 0.0
            # print("Distance travelled in x and z and time_diff", dist_x, dist_z, time_diff)
            # print("speed in x and z", vx, vz) 


                    
            # #publish velocity
            # self.pub_vel_x.publish(vx)
            # self.pub_vel_y.publish(vy)
            # self.pub_vel_z.publish(vz)
            
            # kalman filtering 
            # kal_fil(meanx,meany, meanz,vx,vy,vz, acc_x,acc_y,acc_z)
            
           
        
        
 
            
    

#****************************************************************************************************************

# def kal_fil(mean__x,  mean__y,  mean__z, vel_x,  vel_y, vel_z,accx,accy,accz):
    
        
        Human_prediction = MarkerArray()
        
        
        P = 100.0*np.eye(9) #covariance matrix


        dt = 0.2 #time step = 20 hz
        acc = 1/2.0*dt**2
        vel = dt

        # x[k+1] = A * x[k] where A = dynamic matrix

        A = np.matrix([[1.0, 0.0, 0.0, vel, 0.0, 0.0, acc, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0,  vel, 0.0, 0.0, acc, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0,  vel, 0.0, 0.0, acc],
                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  vel, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  vel, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  vel],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

        print("A SHAPE = ",A.shape) # 9x9

        #y[k] = H * x[k] where H = measurement matrix

        H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

        print("H SHAPE = ",H.shape) # 3x9 #print(H, H.shape)

        rp = 1.0**2  # Noise of Position Measurement
        R = np.matrix([[rp, 0.0, 0.0],
                    [0.0, rp, 0.0],
                    [0.0, 0.0, rp]])

        print("R SHAPE = ", R.shape) # 3x3 #print("R SHAPE = ",R, R.shape)

        #----------------------------------------------------------------------------------------------

        """"

        link: https://github.com/balzer82/Kalman/blob/master/Kalman-Filter-CA-Ball.ipynb?create=1

        The Position of an object can be influenced by a force (e.g. wind),
        which leads to an acceleration disturbance (noise). 
        This process noise has to be modeled with the process noise covariance matrix Q.

        To easily calcualte Q, one can ask the question: 
        How the noise effects my state vector? For example, 
        how the jerk change the position over one timestep dt. 
        With  as the magnitude of the standard deviation of the jerk, which distrubs the ball in 3D space. 
        We do not assume cross correlation, which means if a jerk will act in x direction of the movement, 
        it will not push in y or z direction.

        We can construct the values with the help of a matrix Gs, which is an "actor" to the state vector.
        """

        
        printing.init_printing()
        dts = Symbol('\Delta t')
        Gs = Matrix([dts**3/6, dts**2/2, dts]) #Gs applies effect of an unknown input to state vector

        sj = 0.1 #sigma standard deviation

        #Q = Gs * Gs T * sigma^2

        Q = np.matrix([[(dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6, 0, 0],
                    [0, (dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6, 0],
                    [0, 0, (dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6],
                    [(dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2, 0, 0],
                    [0, (dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2, 0],
                    [0, 0, (dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2],
                    [(dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2), 0, 0],
                    [0, (dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2), 0],
                    [0, 0, (dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2)]]) *sj**2

        print("Q SHAPE = ",Q.shape) # 9x9

        #-----------------------------------------------------------------------------------------------

        # B = disturbance control matrix 

        B = np.matrix([[0.0],
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0],
                    [0.0]])

        print("B SHAPE = ", B.shape) # 9x1 #print("B SHAPE = ",B, B.shape)

        # u = control input

        u = 0.0 # Assumed constant over time

        I = np.eye(9) #identity matrix
        print("I SHAPE = ", I.shape) #print("I SHAPE = ",I, I.shape)

        # MEASUREMENTS Synthetically creation of the Position Data for the ball-----------------------------------------

        Hz = 5.0 # Frequency of Vision System approx 5 for lidar
        dt = 1.0/Hz
        T = 20.0 # s measurement time 1.0s
        m = int(T/dt) # number of measurements

        px= meanx # x Position Start
        py= meany # y Position Start
        pz= meanz # z Position Start

        # vx = vel_x # m/s Velocity at the beginning
        # vy = vel_y # m/s Velocity
        # vz = vel_z # m/s Velocity

        c = 0.0 # Drag Resistance Coefficient
        d = 0.0 # Damping

        Xr=[]
        Yr=[]
        Zr=[]
        for i in range(int(m)):
            #accx = -c*vx**2  # Drag Resistance = 0
            
            vx += acc_x*dt
            px += vx*dt

            #accz = -9.806 + c*vz**2 # Gravitation
            vz += acc_z*dt
            pz += vz*dt
            
            if pz<0.01:
                vz=-vz*d
                pz+=0.02
            if vx<0.1:
                acc_x=0.0
                acc_z=0.0
                
            Xr.append(px)
            Yr.append(py)
            Zr.append(pz)
            
        #ADD NOISE TO REAL POSITION-----------------------------------------------------------------------------

        sp= 0.1 # Sigma for position noise

        Xm = Xr + sp * (np.random.randn(m))
        Ym = Yr + sp * (np.random.randn(m))
        Zm = Zr + sp * (np.random.randn(m))

        #PLOT ----human trajectory observed from cv system-----------------------------------------------

        fig = plt.figure(figsize=(16,9))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(Xm, Zm, Ym, c='gray')
        ax.set_xlabel('X')
        ax.set_ylabel('Z')
        ax.set_zlabel('Y')
        plt.title('human motion trajectory')

        #ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

        # Axis equal
        max_range = np.array([Xm.max()-Xm.min(), Ym.max()-Ym.min(), Zm.max()-Zm.min()]).max() / 3.0
        meanx = Xm.mean()
        meany = Ym.mean()
        meanz = Zm.mean()
        ax.set_xlim(meanx - max_range, meanx + max_range)
        ax.set_ylim(meany - max_range, meany + max_range)
        ax.set_zlim(meanz - max_range, meanz + max_range)
        #plt.savefig('BallTrajectory-Computervision.png', dpi=150, bbox_inches='tight')

        #---------------------------------------------------------------------------------------------------

        measurements = np.vstack((Xm,Ym,Zm))
        # print(measurements.shape) # 3 x 100

        #initial state:

        x = np.matrix([px, py, pz, vx, vy, vz, 0.0, 0.0, 9.81]).T #check accx ; acceleration in z
        print("X SHAPE = ", x.shape) # 9x1 #print(x, x.shape)


        # Preallocation for Plotting
        xt = []
        yt = []
        zt = []
        dxt= []
        dyt= []
        dzt= []
        ddxt=[]
        ddyt=[]
        ddzt=[]
        Zx = []
        Zy = []
        Zz = []
        Px = []
        Py = []
        Pz = []
        Pdx= []
        Pdy= []
        Pdz= []
        Pddx=[]
        Pddy=[]
        Pddz=[]
        Kx = []
        Ky = []
        Kz = []
        Kdx= []
        Kdy= []
        Kdz= []
        Kddx=[]
        Kddy=[]
        Kddz=[]


        #implement equations------------------------------------------------------------------------------

        hitplate=False
        for filterstep in range(m):
            
            # Model the direction switch, when hitting the plate
            if x[2]<0.01 and not hitplate:
                x[5]=-x[5]
                hitplate=True
            
            # Time Update (Prediction)
            # ========================
            # Project the state ahead
            x = A*x + B*u
            
            # Project the error covariance ahead
            P = A*P*A.T + Q   
            # print("P SHAPE = ",P.shape) # 9x9
            
            
            # Measurement Update (Correction)
            # ===============================
            # Compute the Kalman Gain
            S = H*P*H.T + R
            K = (P*H.T) * np.linalg.pinv(S)

            
            # Update the estimate via z
            Z = measurements[:,filterstep].reshape(H.shape[0],1)
            y = Z - (H*x)                            # Innovation or Residual
            x = x + (K*y)
            
            # Update the error covariance
            P = (I - (K*H))*P
            
        
            
            # Save states for Plotting
            xt.append(float(x[0]))
            yt.append(float(x[1]))
            zt.append(float(x[2]))
            dxt.append(float(x[3]))
            dyt.append(float(x[4]))
            dzt.append(float(x[5]))
            ddxt.append(float(x[6]))
            ddyt.append(float(x[7]))
            ddzt.append(float(x[8]))
            
            Zx.append(float(Z[0]))
            Zy.append(float(Z[1]))
            Zz.append(float(Z[2]))
            Px.append(float(P[0,0]))
            Py.append(float(P[1,1]))
            Pz.append(float(P[2,2]))
            Pdx.append(float(P[3,3]))
            Pdy.append(float(P[4,4]))
            Pdz.append(float(P[5,5]))
            Pddx.append(float(P[6,6]))
            Pddy.append(float(P[7,7]))
            Pddz.append(float(P[8,8]))
            Kx.append(float(K[0,0]))
            Ky.append(float(K[1,0]))
            Kz.append(float(K[2,0]))
            Kdx.append(float(K[3,0]))
            Kdy.append(float(K[4,0]))
            Kdz.append(float(K[5,0]))
            Kddx.append(float(K[6,0]))
            Kddy.append(float(K[7,0]))
            Kddz.append(float(K[8,0]))
            
            for i in xt:
                # Human_prediction.header.frame_id = "camera_rgb_optical_frame"
                # Human_prediction.header.stamp = rospy.Time.now()
                # Human_prediction.ns = "basic_shapes"
                Human_prediction.id = 6
                Human_prediction.type = 6
                Human_prediction.pose.position.x =  xt[i]
                Human_prediction.pose.position.y = yt[i]
                Human_prediction.pose.position.z = zt[i]
                Human_prediction.pose.orientation.x = 1.0
                Human_prediction.pose.orientation.y = 1.0
                Human_prediction.pose.orientation.z = 0.0
                Human_prediction.pose.orientation.w = 0.0
                Human_prediction.scale.x = 0.8
                Human_prediction.scale.y = 0.8
                Human_prediction.scale.z = 0.8
                Human_prediction.color.a = 1.0
                Human_prediction.color.r = 0.0
                Human_prediction.color.g = 1.0
                Human_prediction.color.b = 0.0
                
                self.human_pred.publish(Human_prediction)
            
                
            
            
           
            
            # while not rospy.is_shutdown():
            
            
           
           
            
            
            
            #-------------------------------------------------------------
            
            #plot position in 3d-----------------------------------------------------------------------------
       
        fig = plt.figure(figsize=(16,9))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(xt,zt,yt, label='Kalman Filter Estimate')
        ax.plot(Xr, Zr, Yr, label='Real')
        ax.set_xlabel('X')
        ax.set_ylabel('Z')
        ax.set_zlabel('Y')
        ax.legend()
        plt.title('Human Trajectory estimated with Kalman Filter')
        
        

        # Axis equal
        max_range = np.array([Xm.max()-Xm.min(), Ym.max()-Ym.min(), Zm.max()-Zm.min()]).max() / 3.0
        mean__x = Xm.mean()
        mean__y = Ym.mean()
        mean__z = Zm.mean()
        ax.set_xlim(mean__x - max_range, mean__x + max_range)
        ax.set_ylim(mean__y - max_range, mean__y + max_range)
        ax.set_zlim(mean__z - max_range, mean__z + max_range)
        plt.savefig('Kalman-Filter-Human-Trajectory.png', dpi=150, bbox_inches='tight')


        dist = np.sqrt((Xm-xt)**2 + (Ym-yt)**2 + (Zm-zt)**2)
        print('Estimated Position is %.2fm away from human position.' % dist[-1])           
    
               

def main():
    rospy.init_node('Gaussian_DBSCAN', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
    
 
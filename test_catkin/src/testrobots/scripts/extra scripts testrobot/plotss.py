
# #plot initial cov matrix P -----------------------------------------------------------------------------------------
# fig = plt.figure(figsize=(6, 6))
# im = plt.imshow(P, interpolation="none", cmap=plt.get_cmap('binary'))
# plt.title('Initial Covariance Matrix $P$')
# ylocs, ylabels = plt.yticks()
# # set the locations of the yticks
# plt.yticks(np.arange(10))
# # set the locations and labels of the yticks
# plt.yticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# xlocs, xlabels = plt.xticks()
# # set the locations of the yticks
# plt.xticks(np.arange(7))
# # set the locations and labels of the yticks
# plt.xticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# plt.xlim([-0.5,8.5])
# plt.ylim([8.5, -0.5])

# from mpl_toolkits.axes_grid1 import make_axes_locatable
# divider = make_axes_locatable(plt.gca())
# cax = divider.append_axes("right", "5%", pad="3%")
# plt.colorbar(im, cax=cax)


# plt.tight_layout()

# #-----------------------------------------------------------------------------------------------

# #plot measurement noise cov matrix R--------------------------------------------------------------------------------------

# fig = plt.figure(figsize=(4, 4))
# im = plt.imshow(R, interpolation="none", cmap=plt.get_cmap('binary'))
# plt.title('Measurement Noise Covariance Matrix $R$')
# ylocs, ylabels = plt.yticks()
# # set the locations of the yticks
# plt.yticks(np.arange(4))
# # set the locations and labels of the yticks
# plt.yticks(np.arange(3),('$x$', '$y$', '$z$'), fontsize=22)

# xlocs, xlabels = plt.xticks()
# # set the locations of the yticks
# plt.xticks(np.arange(4))
# # set the locations and labels of the yticks
# plt.xticks(np.arange(3),('$x$', '$y$', '$z$'), fontsize=22)

# plt.xlim([-0.5,2.5])
# plt.ylim([2.5, -0.5])

# from mpl_toolkits.axes_grid1 import make_axes_locatable
# divider = make_axes_locatable(plt.gca())
# cax = divider.append_axes("right", "5%", pad="3%")
# plt.colorbar(im, cax=cax)

# plt.tight_layout()

# #-------------------------------------------------------------------------------------------------

# # plot process noise covariance matrix Q--------------------------------------------------------------------
# fig = plt.figure(figsize=(6, 6))
# im = plt.imshow(Q, interpolation="none", cmap=plt.get_cmap('binary'))
# plt.title('Process Noise Covariance Matrix $Q$')
# ylocs, ylabels = plt.yticks()
# # set the locations of the yticks
# plt.yticks(np.arange(10))
# # set the locations and labels of the yticks
# plt.yticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# xlocs, xlabels = plt.xticks()
# # set the locations of the yticks
# plt.xticks(np.arange(7))
# # set the locations and labels of the yticks
# plt.xticks(np.arange(9),('$x$', '$y$', '$z$', '$\dot x$', '$\dot y$', '$\dot z$', '$\ddot x$', '$\ddot y$', '$\ddot z$'), fontsize=22)

# plt.xlim([-0.5,8.5])
# plt.ylim([8.5, -0.5])

# from mpl_toolkits.axes_grid1 import make_axes_locatable
# divider = make_axes_locatable(plt.gca())
# cax = divider.append_axes("right", "5%", pad="3%")
# plt.colorbar(im, cax=cax)


# plt.tight_layout()

# #--------------------------------------------------------------------------------------------

 # #plot position in 3d-----------------------------------------------------------------------------
       
        # fig = plt.figure(figsize=(16,9))
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(xt,zt,yt, label='Kalman Filter Estimate')
        # ax.plot(Xr, Zr, Yr, label='Real')
        # ax.set_xlabel('X')
        # ax.set_ylabel('Z')
        # ax.set_zlabel('Y')
        # ax.legend()
        # plt.title('Human Trajectory estimated with Kalman Filter')
        
        

        # # Axis equal
        # max_range = np.array([Xm.max()-Xm.min(), Ym.max()-Ym.min(), Zm.max()-Zm.min()]).max() / 3.0
        # mean__x = Xm.mean()
        # mean__y = Ym.mean()
        # mean__z = Zm.mean()
        # ax.set_xlim(mean__x - max_range, mean__x + max_range)
        # ax.set_ylim(mean__y - max_range, mean__y + max_range)
        # ax.set_zlim(mean__z - max_range, mean__z + max_range)
        # plt.savefig('Kalman-Filter-Human-Trajectory.png', dpi=150, bbox_inches='tight')
        
        
        
        #***********************************main code*******************
        
        
        
#         import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from scipy.stats import norm
# from sympy import Symbol, Matrix
# from sympy.interactive import printing

# def kal_fil(mean__x,  mean__y,  mean__z, vel_x,  vel_y, vel_z,accx,accy,accz, dt):
    
        
#         P = 100.0*np.eye(9) #covariance matrix 100      
#         print("dt:", dt)

#         # x[k+1] = F * x[k] where F = dynamic matrix

        
#         F = np.matrix([[1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2, 0.0, 0.0],
#                        [0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2, 0.0],
#                        [0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2],
#                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0],
#                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0],
#                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt],
#                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
#                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
#                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

#         # print("A SHAPE = ",A.shape) # 9x9

#         #y[k] = H * x[k] where H = measurement matrix

#         H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#                        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

#         # print("H SHAPE = ",H.shape) # 3x9 #print(H, H.shape)

#         rp = 1.0**2  # Noise of Position Measurement
#         R = np.matrix([[rp, 0.0, 0.0],
#                        [0.0, rp, 0.0],
#                        [0.0, 0.0, rp]])

#         # print("R SHAPE = ", R.shape) # 3x3 #print("R SHAPE = ",R, R.shape)

#         #----------------------------------------------------------------------------------------------

       
#         #Q = Gs * Gs T * sigma^2
       
#         Gs = Matrix([dt**3/6, dt**2/2, dt]) #Gs applies effect of an unknown input to state vector

#         sj = 0.1 #sigma standard deviation 0.1       

#         Q = np.matrix([[(dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6, 0, 0],
#                        [0, (dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6, 0],
#                        [0, 0, (dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6],
#                        [(dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2, 0, 0],
#                        [0, (dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2, 0],
#                        [0, 0, (dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2],
#                        [(dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2), 0, 0],
#                        [0, (dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2), 0],
#                        [0, 0, (dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2)]]) *sj**2

#         # print("Q SHAPE = ",Q.shape) # 9x9

#         #-----------------------------------------------------------------------------------------------

#         # B = disturbance control matrix 

#         B = np.matrix([[0.0],
#                     [0.0],
#                     [0.0],
#                     [0.0],
#                     [0.0],
#                     [0.0],
#                     [0.0],
#                     [0.0],
#                     [0.0]])

#         # print("B SHAPE = ", B.shape) # 9x1 #print("B SHAPE = ",B, B.shape)

#         # u = control input

#         u = 0.0 # Assumed constant over time

#         I = np.eye(9) #identity matrix
#         # print("I SHAPE = ", I.shape) #print("I SHAPE = ",I, I.shape)

#         # MEASUREMENTS Synthetically creation of the Position Data -----------------------------------------

#         # Hz = 5.0 # Frequency of Vision System approx 5 for lidar
#         # dt = 1/Hz #1.0/Hz
        
#         T = 0.7 # s measurement time 1.0s
#         m = int(T/dt) # number of measurements

#         px= mean__x # x Position Start
#         py= mean__y # y Position Start
#         pz= mean__z # z Position Start

#         vx = vel_x # m/s Velocity at the beginning
#         vy = vel_y # m/s Velocity
#         vz = vel_z # m/s Velocity

       
#         Xr=[]
#         Yr=[]
#         Zr=[]
#         for i in range(int(m)):
            
            
#             # vx += accx*dt
#             # px += vx*dt
           
#             # vz += accz*dt
#             # pz += vz*dt
        
                
#             Xr.append(px)
#             Yr.append(py)
#             Zr.append(pz)
            
#         #ADD NOISE TO REAL POSITION-----------------------------------------------------------------------------

#         sp= 0.0 # Sigma for position noise 0.1

#         Xm = Xr + sp * (np.random.randn(m))
#         Ym = Yr + sp * (np.random.randn(m))
#         Zm = Zr + sp * (np.random.randn(m))

       
#         #---------------------------------------------------------------------------------------------------

#         measurements = np.vstack((Xm,Ym,Zm))
#         # print(measurements.shape) # 3 x 100
        

#         #initial state:
#         x = np.matrix([px, py, pz, vx, vy, vz, accx, accy, accz]).T 
#         # print("X SHAPE = ", x.shape) # 9x1 #print(x, x.shape)


#         # Preallocation for Plotting
#         xt = []
#         yt = []
#         zt = []
#         # dxt= []
#         # dyt= []
#         # dzt= []
#         # ddxt=[]
#         # ddyt=[]
#         # ddzt=[]
#         # Zx = []
#         # Zy = []
#         # Zz = []
#         # Px = []
#         # Py = []
#         # Pz = []
#         # Pdx= []
#         # Pdy= []
#         # Pdz= []
#         # Pddx=[]
#         # Pddy=[]
#         # Pddz=[]
#         # Kx = []
#         # Ky = []
#         # Kz = []
#         # Kdx= []
#         # Kdy= []
#         # Kdz= []
#         # Kddx=[]
#         # Kddy=[]
#         # Kddz=[]


#         #implement equations------------------------------------------------------------------------------

   
#         for filterstep in range(m):
            
#             x = F*x + B*u # time update
#             P = F*P*F.T + Q # project error covariance
#             S = H*P*H.T + R
#             K = (P*H.T) * np.linalg.pinv(S) # calculate kalman gain
#             Z = measurements[:,filterstep].reshape(H.shape[0],1) # update estimate
#             y = Z - (H*x)                           
#             x = x + (K*y)
#             P = (I - (K*H))*P # update error covariance
            
           
            
#             # Save states for Plotting
#             xt.append(float(x[0]))
#             yt.append(float(x[1]))
#             zt.append(float(x[2]))
#             # dxt.append(float(x[3]))
#             # dyt.append(float(x[4]))
#             # dzt.append(float(x[5]))
#             # ddxt.append(float(x[6]))
#             # ddyt.append(float(x[7]))
#             # ddzt.append(float(x[8]))            
#             # Zx.append(float(Z[0]))
#             # Zy.append(float(Z[1]))
#             # Zz.append(float(Z[2]))
#             # Px.append(float(P[0,0]))
#             # Py.append(float(P[1,1]))
#             # Pz.append(float(P[2,2]))
#             # Pdx.append(float(P[3,3]))
#             # Pdy.append(float(P[4,4]))
#             # Pdz.append(float(P[5,5]))
#             # Pddx.append(float(P[6,6]))
#             # Pddy.append(float(P[7,7]))
#             # Pddz.append(float(P[8,8]))
#             # Kx.append(float(K[0,0]))
#             # Ky.append(float(K[1,0]))
#             # Kz.append(float(K[2,0]))
#             # Kdx.append(float(K[3,0]))
#             # Kdy.append(float(K[4,0]))
#             # Kdz.append(float(K[5,0]))
#             # Kddx.append(float(K[6,0]))
#             # Kddy.append(float(K[7,0]))
#             # Kddz.append(float(K[8,0]))
           
            
       


#         dist = np.sqrt((Xm-xt)**2 + (Ym-yt)**2 + (Zm-zt)**2)
#         print('Estimated Position is %.2fm away from human position.' % dist[-1])    
        
#         # print ("real x position is:", Xr)
#         # print ("real y position is:", Yr)
#         # print ("real z position is:", Zr)
#         # print ("estimate x position is:", Xm)
#         # print ("estimate y position is:", Ym)
#         # print ("estimate z position is:", Zm)
        
#         return xt,yt, zt, Xr, Yr, Zr , dist    
    
 
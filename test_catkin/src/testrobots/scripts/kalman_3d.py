import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import norm
from sympy import Symbol, Matrix
from sympy.interactive import printing

def kal_fil(prev_x, prev_y, prev_z,curr_x,  curr_y,  curr_z, vel_x,  vel_y, vel_z,accx,accy,accz, dt):
    
        
        P = 10.0*np.eye(9) #covariance matrix 100    
        # print("dt:", dt)
        

        # x[k+1] = F * x[k] where F = dynamic matrix
        # print("current x:", curr_x)
        # print("prev: ", prev_x)

        
        F = np.matrix([[1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2, 0.0, 0.0],
                       [0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2, 0.0],
                       [0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2],
                       [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

        # print("F SHAPE = ",F.shape) # 9x9

        #y[k] = H * x[k] where H = measurement matrix

        H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

        # print("H SHAPE = ",H.shape) # 3x9 #print(H, H.shape)


        # R = Measurement noise variance 
        rp = 0.1#**2  
        R = np.matrix([[rp, 0.0, 0.0],
                       [0.0, rp, 0.0],
                       [0.0, 0.0, rp]])

        # print("R SHAPE = ", R.shape) # 3x3 #print("R SHAPE = ",R, R.shape)

        #----------------------------------------------------------------------------------------------

        # Q = Process noise variance
        # Q = Gs * Gs T * sigma^2
       
        Gs = Matrix([dt**3/6, dt**2/2, dt]) #Gs applies effect of an unknown input to state vector

        sj = 0.1 #sigma standard deviation 0.1       

        Q = np.matrix([[(dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6, 0, 0],
                       [0, (dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6, 0],
                       [0, 0, (dt**6)/36, 0, 0, (dt**5)/12, 0, 0, (dt**4)/6],
                       [(dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2, 0, 0],
                       [0, (dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2, 0],
                       [0, 0, (dt**5)/12, 0, 0, (dt**4)/4, 0, 0, (dt**3)/2],
                       [(dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2), 0, 0],
                       [0, (dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2), 0],
                       [0, 0, (dt**4)/6, 0, 0, (dt**3)/2, 0, 0, (dt**2)]]) *sj**2

        # print("Q SHAPE = ",Q.shape) # 9x9

      
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

        # print("B SHAPE = ", B.shape) # 9x1 #print("B SHAPE = ",B, B.shape)

        # u = control input

        u = 0.0 # Assumed constant over time

        I = np.eye(9) #identity matrix
        # print("I SHAPE = ", I.shape) #print("I SHAPE = ",I, I.shape)

        
        T = 30.0 # 15s measurement time 1.0s
        m = int(T/dt) # number of measurements

        px= prev_x # x Position Start
        py= prev_y # y Position Start
        pz= prev_z # z Position Start

       
            
        # add noise to real pos -----------------------------------------------------------------------------

        sp= 0.0 # Sigma for position noise 0.1

        
        
        Xm = curr_x + sp * (np.random.randn(m))
        Ym = curr_y + sp * (np.random.randn(m))
        Zm = curr_z + sp * (np.random.randn(m))

      

        #---------------------------------------------------------------------------------------------------

        measurements = np.vstack((Xm,Ym,Zm))
        # print(measurements.shape) # 3 x 100
        

        #initial state:
        x = np.matrix([px, py, pz, vel_x, vel_y, vel_z, accx, accy, accz]).T 
        # print("X SHAPE = ", x.shape) # 9x1 #print(x, x.shape)


        # Preallocation 
        xt = []
        yt = []
        zt = []
       


        #implement equations------------------------------------------------------------------------------

   
        for filterstep in range(m):
            
            #prediction
            # print("x:", x)
            # print()
            x = F*x + B*u # time update
            # print("f:", F)
            # print()
            P = F*P*F.T + Q # project error covariance
            # print("P:", P)
            # print()
            
            #correction
            S = H*P*H.T + R
            K = (P*H.T) * np.linalg.pinv(S) # calculate kalman gain
            Z = measurements[:,filterstep].reshape(H.shape[0],1) # update estimate
            y = Z - (H*x)                         
            x = x + (K*y)
            # print("x:", x)
            # print()
            P = (I - (K*H))*P # update error covariance  
            # print("P:", P)
            # print()         
           
            
            # Save states 
        
            xt.append(float(x[0]))
            yt.append(float(x[1]))
            zt.append(float(x[2]))
       
       

        # print("xt:", xt)
        # print()
        # print("yt:", yt)
        # print()
        # print("zt:", zt)
        # print()
        # print("xm:", Xm)
        # print()
        
        
        dist = np.sqrt((Xm-xt)**2 + (Ym-yt)**2 + (Zm-zt)**2)
        # print('Estimated Position is %.2fm away from human position.' % dist[-1])    
  
        
        return xt,yt, zt, dist    
    
 
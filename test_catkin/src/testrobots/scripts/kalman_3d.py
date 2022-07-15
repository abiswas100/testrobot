import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import norm
from sympy import Symbol, Matrix
from sympy.interactive import printing

def kal_fil(mean__x,  mean__y,  mean__z, vel_x,  vel_y, vel_z,accx,accy,accz):
    
        
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

        # print("A SHAPE = ",A.shape) # 9x9

        #y[k] = H * x[k] where H = measurement matrix

        H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

        # print("H SHAPE = ",H.shape) # 3x9 #print(H, H.shape)

        rp = 1.0**2  # Noise of Position Measurement
        R = np.matrix([[rp, 0.0, 0.0],
                    [0.0, rp, 0.0],
                    [0.0, 0.0, rp]])

        # print("R SHAPE = ", R.shape) # 3x3 #print("R SHAPE = ",R, R.shape)

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

        # print("Q SHAPE = ",Q.shape) # 9x9

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

        # print("B SHAPE = ", B.shape) # 9x1 #print("B SHAPE = ",B, B.shape)

        # u = control input

        u = 0.0 # Assumed constant over time

        I = np.eye(9) #identity matrix
        # print("I SHAPE = ", I.shape) #print("I SHAPE = ",I, I.shape)

        # MEASUREMENTS Synthetically creation of the Position Data for the ball-----------------------------------------

        Hz = 20.0 # Frequency of Vision System approx 5 for lidar
        dt = 1.0/Hz
        T = 20.0 # s measurement time 1.0s
        m = int(T/dt) # number of measurements

        px= mean__x # x Position Start
        py= mean__y # y Position Start
        pz= mean__z # z Position Start

        vx = vel_x # m/s Velocity at the beginning
        vy = vel_y # m/s Velocity
        vz = vel_z # m/s Velocity

        c = 0.0 # Drag Resistance Coefficient
        d = 0.0 # Damping

        Xr=[]
        Yr=[]
        Zr=[]
        for i in range(int(m)):
            #accx = -c*vx**2  # Drag Resistance = 0
            
            vx += accx*dt
            px += vx*dt

            #accz = -9.806 + c*vz**2 # Gravitation
            vz += accz*dt
            pz += vz*dt
            
            if pz<0.01:
                vz=-vz*d
                pz+=0.02
            if vx<0.1:
                accx=0.0
                accz=0.0
                
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
        ax.set_xlim(mean__x - max_range, mean__x + max_range)
        ax.set_ylim(mean__y - max_range, mean__y + max_range)
        ax.set_zlim(mean__z - max_range, mean__z + max_range)
        #plt.savefig('BallTrajectory-Computervision.png', dpi=150, bbox_inches='tight')

        #---------------------------------------------------------------------------------------------------

        measurements = np.vstack((Xm,Ym,Zm))
        # print(measurements.shape) # 3 x 100

        #initial state:

        x = np.matrix([px, py, pz, vx, vy, vz, 0.0, 0.0, 9.81]).T #check accx ; acceleration in z
        # print("X SHAPE = ", x.shape) # 9x1 #print(x, x.shape)


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
        
        print ("real x position is:", Xr)
        print ("real y position is:", Yr)
        print ("real z position is:", Zr)
        print ("estimate x position is:", Xm)
        print ("estimate y position is:", Ym)
        print ("estimate z position is:", Zm)
        
        return xt,yt, zt, Xr, Yr, Zr      
    
 
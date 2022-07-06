#!/usr/bin/env python
#!/usr/bin/env python3

#imports 



import numpy as np

class KF(object):

    def __init__(self,initial_x:float,initial_v:float,acc_variance:float):
        
    
        
        # mean of state GRV
        self._x = np.array([initial_x,initial_v])
        self._acc_variance = acc_variance
        
        #covariance of state GRV
        self._p = np.eye(2)
        

        
    def predict(self,dt:float)-> None:
        
        # x = F  x
        # P = F  P  Ft + G  Gt a
        
        F = np.array([[1,dt],[0,1]])
        new_x = F.dot(self._x)
        
        G = np.array([0.5 * dt**2, dt]).reshape((2,1))
        new_P = F.dot(self._p).dot(F.T) + G.dot(G.T) * self._acc_variance
        
        self._x = new_x
        self._p = new_P
        
    def update(self, meas_value:float, meas_variance:float):
        
        # y = z - H x
        # S = H P Ht + R
        # K = P Ht S^ -1
        # x = x + K y
        # P = (I -K H) * P
        
        H = np.array([1,0]).reshape((1,2))
      
        
        z = np.array([meas_value])
        R = np.array([meas_variance])
        
        y = z - H.dot(self._x)
        S = H.dot(self._p).dot(H.T) + R
        
        K = self._p.dot(H.T).dot(np.linalg.inv(S))
        
        new_X = self._x + K.dot(y)
        new_P = (np.eye(2) - K.dot(H)).dot(self._p)
        
        self._p = new_P
        self._x = new_X
        
       
        
    @property
    def pos(self)-> float:
        return self._x[0]
         
        # @property
        # def pos_z(self)-> float:
        #     return self._x[1]
        
    @property
    def vel(self)-> float:
        return self._x[1]
        
    @property
    def cov(self) -> np.array:
        return self._p
        
    @property
    def mean(self) -> np.array:
        return self._x
        
                    
      
        
# def main():
    

    
    
    
    
    
    # notes for avhishek:
    # 1) _x should be [initial_x,initial_v] as per vdo but we dont have v yet so figure that out 
    # 2) i have added initial_z also for the z coordinate
    # 3) G and F matrices are 2x2 but we need 3x3 if we add z coordinate as well
    # 4) I dont know values for G and F if it is 3x3, vdo shows only for 2x2 ughhhh
    # 5) implemented till 21:00 of the vdo 
    # 6) add update function
    # 6) dont know how to find dt (delta t = time step)
    # 7) havent tried running the code yet for obvious reasons duhhhhh
    # 8) added file name scripts/kf.py to cmakelists
    # 9) do not otch gaussian3d. Everything is copied from there. work on this script for now
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
 
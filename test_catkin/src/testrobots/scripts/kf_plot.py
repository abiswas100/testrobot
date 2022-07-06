import numpy as np
import matplotlib.pyplot as plt
from kf import KF

plt.ion()
plt.figure()

kf = KF(initial_x=0.0,initial_v=1.0,acc_variance=0.1)

DT = 0.1
NUM_STEPS = 1000
meas_every_step = 20

mus = []
covs = []

real_x = 0.0
real_v = 0.9
mea_variance = 0.1 ** 2
real_xs = []
real_vs = []


for step in range(NUM_STEPS):
    covs.append(kf.cov)
    mus.append(kf.mean)
    
    real_x = real_x + DT * real_v
    
    kf.predict(dt=DT)
    
    
    if step != 0 and step % meas_every_step == 0:
        kf.update(meas_value=real_x + np.random.randn() * np.sqrt(mea_variance), 
                  meas_variance=mea_variance)
        
    real_xs.append(real_x)
    real_vs.append(real_v)
    
##for plotting

plt.subplot(2,1,1)
plt.title('position')
plt.plot([mu[0] for mu in mus], 'r')
plt.plot(real_xs,'b')
plt.plot([mu[0] - 2*np.sqrt(cov[0,0])for mu,cov in zip(mus,covs)],'r--')
plt.plot([mu[0] + 2*np.sqrt(cov[0,0])for mu,cov in zip(mus,covs)],'r--')

plt.subplot(2,1,2)
plt.title('velocity')
plt.plot([mu[1] for mu in mus], 'r')
plt.plot(real_vs,'b')
plt.plot([mu[1] - 2*np.sqrt(cov[1,1])for mu,cov in zip(mus,covs)],'r--')
plt.plot([mu[1] + 2*np.sqrt(cov[1,1])for mu,cov in zip(mus,covs)],'r--')


plt.show()
plt.ginput(1)
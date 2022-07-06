import numpy as np
import matplotlib.pyplot as plt
from kf import KF

plt.ion()
plt.figure()

kf = KF(initial_x=0.0,initial_v=1.0,acc_variance=10.0)

DT = 0.1
NUM_STEPS = 1000

mus = []
covs = []

for i in range(NUM_STEPS):
    covs.append(kf.cov)
    mus.append(kf.mean)
    
    kf.predict(dt=DT)

plt.subplot(2,1,1)
plt.title('position')
plt.plot([mu[0] for mu in mus], 'r')
plt.plot([mu[0] - 2*np.sqrt(cov[0,0])for mu,cov in zip(mus,covs)],'r--')
plt.plot([mu[0] + 2*np.sqrt(cov[0,0])for mu,cov in zip(mus,covs)],'r--')

plt.subplot(2,1,2)
plt.title('velocity')
plt.plot([mu[1] for mu in mus], 'r')
plt.plot([mu[1] - 2*np.sqrt(cov[1,1])for mu,cov in zip(mus,covs)],'r--')
plt.plot([mu[1] + 2*np.sqrt(cov[1,1])for mu,cov in zip(mus,covs)],'r--')


plt.show()
plt.ginput(1)
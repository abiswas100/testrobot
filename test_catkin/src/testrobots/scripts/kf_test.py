from matplotlib import colors
from kalman_3d import kal_fil
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt



# first reading ****************************

x1 = -1.1046578075204576 
y1 = 0.0
z1 = 2.4163219928741455 

vx1 = 0.27 
vy1 = 0.0
vz1 = 0.53

accx1 = 1.8 
accy1 = 0.0
accz1 = 3.53

time_diff1 = 0.14999999999417923

# second reading ***************************************

x2 =  -1.140676578337496 
y2 = 0.0
z2 =  2.3391923091628333

vx2 = 0.12 
vy2 = 0.0
vz2 = 0.82

accx2 = 0.71 
accy2 = 0.0
accz2 = 4.82

time_diff2 = 0.17000000001280569


#third reading**********************************************

x3 = -1.1559784809748332 
y3 = 0.0
z3 = 2.197830573717753

vx3 = 0.25 
vy3 = 0.0
vz3 = 0.81

accx3 = 1.56 
accy3 = 0.0
accz3 = 5.06

time_diff3 = 0.15999999997438863



# fourth reading ******************************************** 

x4 = -1.1984011434219979 
y4 = 0.0
z4 = 2.067028793128761



xt1,yt1,zt1,dist1 = kal_fil(x1,y1, z1,x2,y2,z2,vx1,vy1,vz1, accx1,accy1,accz1,time_diff1)
xt2,yt2,zt2,dist2 = kal_fil(x2,y2, z2,x3,y3,z3,vx2,vy2,vz2, accx2,accy2,accz2,time_diff2)
xt3,yt3,zt3,dist3 = kal_fil(x3,y3, z3,x4,y4,z4,vx3,vy3,vz3, accx3,accy3,accz3,time_diff3)

# 2d plot
plt.scatter(xt1,zt1,c="yellow")
plt.scatter(xt2,zt2,c="yellow")
plt.scatter(xt3,zt3,c="yellow")
plt.scatter(x1,z1,c="black")
plt.scatter(x2,z2,c="black")
plt.scatter(x3,z3,c="black")
plt.scatter(x4,z4,c="black")
plt.show()

# 3d plot
# fig = plt.figure(figsize = (10, 7))
# ax = plt.axes(projection ="3d")
# ax.scatter3D(xt1, yt1, zt1, color = "green")
# ax.scatter3D(xt2, yt2, zt2, color = "blue")
# plt.show()



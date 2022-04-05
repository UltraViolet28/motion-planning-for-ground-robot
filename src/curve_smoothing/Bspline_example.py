import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
from rdp import rdp

x = np.array([ 0. ,  1.2,  1.9,  3.2,  4. ,  6.5, 7 ,9.2 , 10, 12 , 13.5])
y = np.array([ 0. ,  2.3,  3. ,  4.3,  2.9,  3.1, 4, 10, 1.5, 3 , 12])
print(len(x))

t, c, k = interpolate.splrep(x, y, s=0, k=4)
N = 100
xmin, xmax = x.min(), x.max()
xx = np.linspace(xmin, xmax, N)
spline = interpolate.BSpline(t, c, k, extrapolate=False)



points = np.column_stack([x, y])
new_pts = rdp(points, epsilon=0.5)
print(len(new_pts))
print(new_pts)
nx ,ny =np.split(new_pts,2,axis=1)
print(nx)
print(ny)

t1, c1, k1 = interpolate.splrep(x, y, s=0, k=4)
N = 100
xmin, xmax = nx.min(), nx.max()
xx1 = np.linspace(xmin, xmax, N)
spline1 = interpolate.BSpline(t1, c1, k1, extrapolate=False)


fig, axs = plt.subplots(2)


axs[0].plot(x, y, 'bo', label='Original points')
axs[0].plot(xx, spline(xx), 'r', label='BSpline')
axs[0].grid()
plt.legend(loc='best')
#plt.show()

axs[1].plot(nx,ny, 'go', label='new_pts')
axs[1].plot(xx1, spline(xx1), 'b', label='BSpline')
axs[1].grid()
axs[1].legend(loc='best')
plt.show()



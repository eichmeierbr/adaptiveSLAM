import math
import matplotlib.pyplot as plt
import numpy as np

# n=100
# r=100
# b=0
# lx = []
# ly = []

# for i in range(-n,n):
#     for j in range(-n,n):
#         if math.sqrt(i**2+j**2)<=r+b and math.sqrt(i**2+j**2)>=r-b:
#             lx.append(i)
#             ly.append(j)

# lx = np.add(lx,300)
# ly = np.add(ly,200)

# print(lx,ly)

def rolling_avg(path):
    pathx = [path[0,0]]
    pathy = [path[0,1]]
    for i in range(1,len(path)-1):
        avg_x = (path[i-1,0]+2*path[i,0]+path[i+1,0])/4
        avg_y = (path[i-1,1]+2*path[i,1]+path[i+1,1])/4

        # pathx.append(path[i,0])
        # pathy.append(path[i,1])

        pathx.append(avg_x)
        pathy.append(avg_y)

    return pathx,pathy

def interpolate(x,y):
    newx = []
    newy = []
    for i in range(len(x)-1):
        newx.append(x[i])
        newy.append(y[i])

        diffx = (x[i+1]-x[i])/2
        diffy = (y[i+1]-y[i])/2

        newx.append(x[i]+diffx)
        newy.append(y[i]+diffy)

    return newx,newy


emptymap_complex = np.array([[100,200,204,220,240,272,300, 328,360,380,396,396,380,360,328,272,240,220,204,200,100],
                                 [100,200, 172,140,120,104,100,104,120,140,172,228,260,280,296,296,280,260,228,200,100]]).T
x,y = rolling_avg(emptymap_complex)
x,y = interpolate(x,y)
x.append(100)
y.append(100)
# x,y = rolling_avg(np.array([x,y])) 
print(x,y)



plt.plot(emptymap_complex[:,0],emptymap_complex[:,1])
plt.plot(x,y)
plt.show()
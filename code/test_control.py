
import control 
import matplotlib.pyplot as plt
import numpy as np


robot1 = control.Robot_movement(0,0)
time = 0.5
plt.axis([0, 10, 0, 10])
path = np.array([[0,1,0,1,0,1,0,1,0,1,0,1],
                 [1,0,1,0,1,0,1,0,1,0,1,0]])

x,y = robot1.get_path()
plt.plot(x,y)
plt.draw()
plt.pause(time)

for i in range(len(path[0,:])):


    robot1.take_step(path[0,i],path[1,i])
    x,y = robot1.get_path()
    plt.plot(x,y)
    plt.draw()
    plt.pause(time)





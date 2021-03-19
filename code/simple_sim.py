import numpy as np

import matplotlib.pyplot as plt
from base_sensor import *
from feature_sensor import *
from robot import *
from control import *

def display_map(env, robot, dt, sensor):
    plt.imshow(env, cmap='Greys')
    plt.scatter(robot._true_pose[0], robot._true_pose[1])
    plt.scatter(robot._est_pose[0], robot._est_pose[1])

    for sens in sensors:
        sensor_vals = sens.get_values()
        if sensor_vals is None:
            continue
        else:
            plt.scatter(sensor_vals[:,0], sensor_vals[:,1])
    

    plt.draw()
    plt.pause(dt)
    plt.clf()

def get_path(waypoints,step_size=10):
    pathx = [waypoints[0,0]]
    pathy = [waypoints[0,1]]

    for i in range(len(waypoints)-1):
        lx = len(pathx)
        ly = len(pathy)

        if pathx[lx-1]<waypoints[i+1,0]:
            pathx = np.append(pathx,np.arange(pathx[lx-1],waypoints[i+1,0],step=step_size))
        else:
            pathx = np.append(pathx,np.flip(np.arange(waypoints[i+1,0],pathx[lx-1],step=step_size)))

        if pathy[ly-1]<waypoints[i+1,1]:
            pathy = np.append(pathy,np.arange(pathy[ly-1],waypoints[i+1,1],step=step_size))
        else:
            pathy = np.append(pathy,np.flip(np.arange(waypoints[i+1,1],pathy[ly-1],step=step_size)))
        lx = len(pathx)
        ly = len(pathy)

        if lx>ly:
            pathy = np.append(pathy,np.ones((lx-ly,1))*pathy[ly-1])

        if ly>lx:
            pathx = np.append(pathx,np.ones((ly-lx,1))*pathx[lx-1])

            


    path = np.zeros((len(pathx),2))
    path[:,0] = pathx
    path[:,1] = pathy

    return path


if __name__ == "__main__":
    
    dt  = 0.0001
    t   = 0.0
    init_pose = np.array([100,100])

    ## Initialize Map
    env = np.loadtxt('../maps/empty_map.csv', delimiter=',')


    ## Initialize Robot
    diff_control = Diff_movement()
    abs_control = Abs_movement()
    robot = Robot(init_pose, env, abs_control)

    ## Initialize Sensors
    sensors = []
    sensors.append(Base_sensor())
    sensors.append(feature_sensor())

    ## Initailze Path
    #this is for loca control vs abs
    # path = np.array([[0,0,0,250,150,0,0,-150,-250,0,0,150],
    #                  [0,100,100,0,0,-100,-100,0,0,100,100,0]]).T

    waypoints = np.array([[100,100,100,350,500,500,500,350,100,100,100,250],
                       [100,200,300,300,300,200,100,100,100,200,300,300]]).T

    path = get_path(waypoints)

    display_map(env, robot, dt, sensors)

    ## For time duration (Or all actions)
    for act in path:

        ## Perform motion with robot
        robot.move(act)

        ## Obtain new sensor measurements
        zs = []
        for sens in sensors:
            zs.append(sens.getMeasure(env, robot))


        ## Do SLAM
        ## TODO: SLAM

        ## Display map
        display_map(env, robot, dt, sensors)



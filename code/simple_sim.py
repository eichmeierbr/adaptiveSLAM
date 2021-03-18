import numpy as np

import matplotlib.pyplot as plt
from base_sensor import *
from robot import *
from control import *

def display_map(env, robot, dt):
    plt.imshow(env, cmap='Greys')
    plt.scatter(robot._true_pose[0], robot._true_pose[1])
    plt.scatter(robot._est_pose[0], robot._est_pose[1])

    plt.draw()
    plt.pause(dt)
    plt.clf()


if __name__ == "__main__":
    
    dt  = 0.1
    t   = 0.0
    init_pose = np.array([100,100])

    ## Initialize Map
    env = np.loadtxt('maps/empty_map.csv', delimiter=',')


    ## Initialize Robot
    diff_control = Diff_movement()
    robot = Robot(init_pose, env, diff_control)

    ## Initialize Sensors
    sensors = []
    sensors.append(Base_sensor())

    ## Initailze Path
    path = np.array([[0,0,0,250,150,0,0,-150,-250,0,0,150],
                     [0,100,100,0,0,-100,-100,0,0,100,100,0]]).T

    display_map(env, robot, dt)

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
        display_map(env, robot, dt)



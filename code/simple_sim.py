import numpy as np

import matplotlib.pyplot as plt
from base_sensor import *
from feature_sensor import *
from robot import *
from odometry import *
from path import *

from math import log

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


if __name__ == "__main__":
    
    dt  = 0.0001
    t   = 0.0
    init_pose = np.array([100,100])

    ## Initialize Map
    env = np.loadtxt('../maps/empty_map.csv', delimiter=',')


    ## Initialize Robot
    diff_control = Diff_movement()
    abs_control = Abs_movement()
    robot = Robot(init_pose, env, diff_control)

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
    global_path = get_path(waypoints)
    local_path = convert_2_local(global_path)

    display_map(env, robot, dt, sensors)

    ## For time duration (Or all actions)
    for act in local_path:
        previous_true_pose = robot._true_pose[:]
        previous_est_pose  = robot._est_pose[:]

        ## Perform motion with robot
        robot.move(act)

        ## Obtain new sensor measurements
        zs = []
        zs.append(robot._odom.get_measure())  ## Add odometry measurement
        for sens in sensors:
            zs.append(sens.getMeasure(env, robot))     ## Add each sensor measurement



        ## Do SLAM
        ## TODO: SLAM

        ## Display map
        display_map(env, robot, dt, sensors)



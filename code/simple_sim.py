import numpy as np

import matplotlib.pyplot as plt
from base_sensor import *
from feature_sensor import *
from robot import *
from odometry import *
from path import *
from enviornment import *
from pathlib import Path

from math import log

def display_map(env, robot, dt, sensor):
    plt.imshow(env.map, cmap='Greys')
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

def select_world(num,step=10,smooth=True):
    if num==1:
        env_map = Path("../maps/empty_map.csv")
        waypoints = np.multiply(np.array([[100, 138.0, 176.0, 191.5, 207.0, 214.0, 221.0, 232.0, 243.0, 257.0, 271.0, 285.5, 300.0, 314.5, 329.0, 343.0, 357.0, 368.0, 379.0, 385.5, 392.0, 392.0, 392.0, 385.5, 379.0, 368.0, 357.0, 339.5, 322.0, 300.0, 278.0, 260.5, 243.0, 232.0, 221.0, 214.0, 207.0, 191.5, 100],
                                          [100, 134.0, 168.0, 169.5, 171.0, 157.0, 143.0, 132.0, 121.0, 114.0, 107.0, 104.5, 102.0, 104.5, 107.0, 114.0, 121.0, 132.0, 143.0, 160.5, 178.0, 200.0, 222.0, 239.5, 257.0, 268.0, 279.0, 285.5, 292.0, 292.0, 292.0, 285.5, 279.0, 268.0, 257.0, 243.0, 229.0, 205.5, 100]]).T,1.25)
    elif num==2:
        env_map = Path("../maps/building_easy.csv")
        waypoints = np.array([[100,100,600,600,100,100],
                              [100,300,300,80 ,80 ,100]]).T
    elif num==3:
        env_map = Path("../maps/sweden_map.csv")
        waypoints = np.array([[100,100,350,350,600,600,100,100],
                              [100,300,300,200,200,80 ,80 ,100]]).T   
    elif num==4:
        env_map = Path("../maps/building.csv")
        waypoints = np.array([[100,100,240,240,400,400,400,100,100],
                              [100,200,200,300,300,350,300,300,100]]).T
    else:
        env_map = Path("../maps/empty_map.csv")
        waypoints = np.array([[100,100,500,500,350,100],
                              [100,300,300,100,100,100]]).T

    waypoints = get_path(waypoints,step_size=step,smooth=smooth)
    waypoints = convert_2_local(waypoints)
    return waypoints,env_map


if __name__ == "__main__":
    
    dt  = 0.0001
    t   = 0.0
    init_pose = np.array([100,100])

    ## Initialize Sensors
    sensors = []
    sensors.append(Base_sensor())
    sensors.append(feature_sensor())

    ## Initialize Map
    local_path,env_map = select_world(4)
    env = Enviornment(sensors,env_map)

    ## Initialize Robot
    diff_control = Diff_movement()
    abs_control = Abs_movement()
    robot = Robot(init_pose, env, diff_control)



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



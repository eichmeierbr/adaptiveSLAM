import numpy as np

import matplotlib.pyplot as plt
from base_sensor import *
from feature_sensor import *
from odom_sensor import *
from gps_sensor import *
from robot import *
from odometry import *
from path import *
from environment import *
from slam_class import *
from pathlib import Path


from math import log

def display_map(env, robot, dt, sensor):
    plt.imshow(env.map, cmap='Greys')
    plt.scatter(robot._true_pose[0], robot._true_pose[1])
    plt.scatter(robot._est_pose[0], robot._est_pose[1])

    # for sens in sensors:
        # sensor_vals = sens.getMeasure(env, robot)
        # if sensor_vals is None:
        #     continue
        # else:
            # plt.scatter(sensor_vals[0], sensor_vals[1],s=0.6)
    

    plt.draw()
    plt.pause(dt)
    plt.clf()




if __name__ == "__main__":
    
    dt  = 0.0001
    t   = 0.0
    init_pose = np.array([100,100])

    ## Initialize Sensors
    sensors = []
    sensors.append(GPS_sensor())
    sensors.append(feature_sensor())
    # sensors.append(Base_sensor())
    sensors.append(odometry_sensor())

    ## Initialize Map and features
    local_path,env,features = select_world(sensors,3, num_features=10)
    sensors[1].set_values(features)  #alex, feel free to change this 

    ## Initialize Robot
    diff_control = Diff_movement()
    abs_control = Abs_movement()
    robot = Robot(init_pose, env, diff_control)

    ## SLAM class
    slammer = Slamma_Jamma()

    display_map(env, robot, dt, sensors)
    idx = 0

    ## For time duration (Or all actions)
    for act in local_path:
        previous_true_pose = robot._true_pose[:]
        previous_est_pose  = robot._est_pose[:]

        ## Perform motion with robot
        robot.move(act)

        ## Obtain new sensor measurements
        zs = []
        # zs.append([0,robot._odom.get_measure()])  ## Add odometry measurement
        i = 0
        for sens in sensors:
            zs.append([i, sens.getMeasure(env, robot)])     ## Add each sensor measurement
            i +=1


        ## Do SLAM
        ## TODO: SLAM
        slammer.record_measurements(idx, zs)
        
        opt_freq = 10
        if idx%opt_freq == opt_freq-1:
            slammer.optimize(robot, sensors)

        ## Display map
        display_map(env, robot, dt, sensors)
        idx += 1



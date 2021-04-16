import numpy as np

import matplotlib.pyplot as plt
from display import *
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
    for sen in sensors:
        if sen._sensor == "feature":
            sensors[1].set_values(features) 

    ## Initialize Robot
    diff_control = Diff_movement()
    abs_control = Abs_movement()
    robot = Robot(init_pose, env, diff_control)

    ## SLAM class
    slammer = Slamma_Jamma()

    # display_map(env, robot, dt, sensors)
    idx = 0
    path_len = len(local_path)
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
        if (idx%opt_freq == opt_freq-1) or (idx-1==path_len):
            slammer.optimize(robot, sensors)

        ## Display map
        ds = display_stuff(robot)
        ds.display_map2(env, robot, dt, zs, sensors)
        idx += 1

        if idx==path_len:
            print("done")



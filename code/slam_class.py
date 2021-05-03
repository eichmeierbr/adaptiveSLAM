import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, minimize

from weighter import *


def parse_xo(xo, measurements, robot, sensors):
        # Parse xo
    num_poses = len(measurements)
    pose_dim = len(robot._est_pose)

    # parse out poses
    poses = xo[:num_poses*pose_dim].reshape([-1, pose_dim])

    # Parse out landmarks
    temp_feats = xo[num_poses*pose_dim:]
    features = []
    for sensor in sensors:
        ## TODO: Is this working for multiple different feature sensors???
        if sensor._num_features > 0:
            num = sensor._num_features
            dim = sensor.feature_dim
            this_feats = temp_feats[:num*dim].reshape([-1, dim])
            features.append(this_feats)
            a=3
        else: features.append([])
    return poses, features


def error_func(xo, measurements, robot, sensors, anchor, last_opt, weights):
    error = np.array([])

    poses, features = parse_xo(xo, measurements, robot, sensors)

    # Compute errors
    for meas_set in measurements:
        t = meas_set[0] - last_opt
        for z in meas_set[1]:
            if len(z[1]) == 0: continue
            args = [features[z[0]]]
            if sensors[z[0]]._sensor == "gps" or sensors[z[0]]._sensor == "feature":
                errs = sensors[z[0]].error_function(poses[t], z[1], args)

                ## TODO: Need smarter way to normalize or weight sensors


            if sensors[z[0]]._sensor == "odometry":
                if t == last_opt-meas_set[0]:
                    errs = sensors[z[0]].error_function(anchor, z[1], [poses[t], robot])
                else:
                    errs = sensors[z[0]].error_function(poses[t-1], z[1], [poses[t], robot])

                # used for midterm testing
                # x,y = robot._true_path[t,:]
                # if x>340 and y>160:
                #     errs = errs*0

            errs = weights[t,z[0]]*errs
            errs /= errs.size
            error = np.hstack((error, np.array(errs).flatten()))                
                
    return error
                

            



class Slamma_Jamma:
    def __init__(self, opt_rate=10, method=0):
        self.measurements = []
        self.last_opt = 0
        self.weighter = weighter()
        self.opt_rate = opt_rate
        self._method = method
        


    def record_measurements(self,idx,zs):
        ## Note: This is a pretty simplistic way to store the data
        ##       We may want to improve this later 
        self.measurements.append([idx, zs])


    def optimize(self, robot, sensors):
        ## Note: This optimization is a simple take parameters and return best guess.

        poses = robot._est_path[self.last_opt+1:]
        anchor = robot._est_path[self.last_opt]
        measurements = self.measurements[self.last_opt:]

        xo = np.array(poses).flatten()

        # Extract out landmarks to optimize
        for sensor in sensors:
            if sensor._sensor == "feature":
                # If the landmarks aren't estimated, estimate them using the first pose
                if len(sensor.features_est) == 0:
                    sensor.estimate_landmark_positions(measurements[0][1][1][1], anchor)
                other = sensor.features_est.flatten()
                xo = np.hstack((xo, other))


        if self._method == 0:
            #### FIRST WEIGHTING OPTION (1 WEIGHT FOR ALL 10)
            weights = self.weighter.get_weights(sensors, measurements,  anchor)

        if self._method == 1:
            #### SECOND WEIGHTING OPTION (SMOOTH OVER 20)
            weights = self.weighter.get_all_weights(sensors, self.measurements, robot._est_path, self.last_opt, self.opt_rate)

        if self._method == -1:
            #### NO WEIGHTING
            weights = np.ones([len(measurements), len(sensors)])

        out = least_squares(error_func, xo, args=([measurements, robot, sensors, anchor, self.last_opt, weights]))
        
        poses, features = parse_xo(out.x, measurements, robot, sensors)


        robot._est_path[self.last_opt+1:] = list(poses)
        robot._est_pose = robot._est_path[-1]

        for i in range(len(features)):
            sensors[i].features_est = features[i]
        
        if self._method == 0 or self._method == -1:
            #### FIRST WEIGHTING OPTION (1 WEIGHT FOR ALL 10)
            self.last_opt = len(self.measurements)

        # if self._method == 1 or self._method == -1:
        if self._method == 1:
            #### SECOND WEIGHTING OPTION (SMOOTH OVER 20)
            if len(self.measurements) > self.opt_rate:
                self.last_opt = len(self.measurements) - self.opt_rate
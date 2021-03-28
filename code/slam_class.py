import numpy as np
from scipy.optimize import least_squares, minimize



def error_func(xo, measurements, robot, sensors):
    error = []

    # Parse xo
    num_poses = len(measurements) + 1
    pose_dim = len(robot._est_pose)

    # parse out poses
    poses = xo[:num_poses*pose_dim].reshape([-1, pose_dim])

    # Parse out landmarks
    temp_feats = xo[num_poses*pose_dim:]
    features = []
    for sensor in sensors:
        if sensor._num_features > 0:
            num = sensor._num_features
            dim = sensor.feature_dim
            this_feats = temp_feats[:num*dim].reshape([-1, dim])
            features.append(this_feats)
            a=3
        else: features.append([])


    # Compute errors
    for meas_set in measurements:
        for z in meas_set[1]:
            # if z[0] == 0:
            a=3

    return 20

class Slamma_Jamma:
    def __init__(self):
        a=3
        self.measurements = []


    def record_measurements(self,idx,zs):
        ## Note: This is a pretty simplistic way to store the data
        ##       We may want to improve this later 
        self.measurements.append([idx, zs])

    def optimize(self, robot, sensors):
        ## Note: This optimization is a simple take parameters and return best guess.

        xo = np.array(robot._est_path).flatten()

        for sensor in sensors:
            if sensor._sensor == "feature":
                other = sensor.features.flatten()
                xo = np.hstack((xo, other))

        out = least_squares(error_func, xo, args=([self.measurements, robot, sensors]))
        a=3

    
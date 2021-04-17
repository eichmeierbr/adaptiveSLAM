import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, minimize


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


def error_func(xo, measurements, robot, sensors, anchor, last_opt):
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
                errs /= errs.size
                # if sensors[z[0]]._sensor == "feature":
                    # errs /=  1000
                error = np.hstack((error, np.array(errs).flatten()))


            if sensors[z[0]]._sensor == "odometry":
                if t == last_opt-meas_set[0]:
                    errs = sensors[z[0]].error_function(anchor, z[1], [poses[t], robot])
                else:
                    errs = sensors[z[0]].error_function(poses[t-1], z[1], [poses[t], robot])

                # used for midterm testing
                # x,y = robot._true_path[t,:]
                # if x>340 and y>160:
                #     errs = errs*0

                error = np.hstack((error, np.array(errs).flatten()))                
                
                a=3

            


    return error

class Slamma_Jamma:
    def __init__(self):
        self.measurements = []
        self.last_opt = 0
        


    def record_measurements(self,idx,zs):
        ## Note: This is a pretty simplistic way to store the data
        ##       We may want to improve this later 
        self.measurements.append([idx, zs])

    def optimize(self, robot, sensors):
        ## Note: This optimization is a simple take parameters and return best guess.

        xo = np.array(robot._est_path[self.last_opt+1:]).flatten()

        for sensor in sensors:
            if sensor._sensor == "feature":
                other = sensor.features.flatten()
                xo = np.hstack((xo, other))

        out = least_squares(error_func, xo, args=([self.measurements[self.last_opt:], robot, sensors, robot._est_path[self.last_opt], self.last_opt]))
        
        poses, features = parse_xo(out.x, self.measurements[self.last_opt:], robot, sensors)

        # pt = np.array(robot._true_path)
        # pe = np.array(robot._est_path)
        # plt.plot(poses[:,0], poses[:,1], label="Optimized")
        # plt.plot(pt[:,0], pt[:,1], label="True")
        # plt.plot(pe[:,0], pe[:,1], label="Est")
        # # plt.show()

        # feats = features[1]
        # tf = sensors[1].features
        # plt.scatter(feats[:,0], feats[:,1])
        # plt.scatter(tf[:,0], tf[:,1])
        # plt.legend()
        # plt.show()

        robot._est_path[self.last_opt+1:] = list(poses)
        robot._est_pose = robot._est_path[-1]
    
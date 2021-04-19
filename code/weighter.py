import numpy as np
from scipy.optimize import least_squares
from sklearn.neural_network import MLPRegressor, MLPClassifier



class weighter():
    def __init__(self):
        self.model = 0

    def get_nominal_paths(self, sensors, zs, anchor):
        est_paths = []

        ## Separate out sensor measurements by sensor
        zs_sensor = {}
        for zt in zs:
            for z in zt[1]:
                if z[0] in zs_sensor:
                    zs_sensor[z[0]].append([zt[0], z[1]])
                else:
                    zs_sensor[z[0]] = [[zt[0], z[1]]]

        ## Estimate a path for each sensor.
        ## NOTE: This assumes a datapoint is given at each time stamp.
        for i, sense in enumerate(sensors):
            z = zs_sensor[i]
            est_paths.append(sense.get_nominal_path(z, args=anchor))
        
        return est_paths

    def getCovarianceMat(self, est_paths):
        s = np.zeros([est_paths.shape[0], est_paths.shape[0]])

        for i in range(est_paths.shape[0]):
            for j in range(est_paths.shape[0]):
                if i == j:
                    continue
                err = est_paths[i] - est_paths[j]
                s[i,j] = np.std(err)

        idxs = np.sort(np.unique(s, return_index=True)[1])[1:]
        return s.flatten()[idxs] 


    def get_weights(self, sensors, zs, anchor):
        est_paths = self.get_nominal_paths(sensors, zs, anchor)
        

        est_paths = np.array(est_paths)
        s = self.getCovarianceMat(est_paths)
        
        return np.ones(len(sensors))




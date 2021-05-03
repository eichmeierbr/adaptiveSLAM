from base_sensor import *
from robot import Robot

# Models GPS as a constant Gaussian Distribution
class GPS_sensor(Base_sensor):
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, mean_error = 5, std = 3, dim = 2, freq=0.01):
        super().__init__(local, in_map, dim, freq)
        self._mean_error = np.zeros(dim) + mean_error # meter
        self._cov = np.eye(dim) * std**2
        self._sensor = "gps"




    # Given true robot state and environment, return modeled GPS measurement
    def getMeasure(self, env, robot):
        # true_pose = robot._true_pose
        # gps_mean = true_pose + self._mean_error
        # return np.random.multivariate_normal(gps_mean, self._cov)

        # applies noise to robot's next true location
        (mean,stddev) = self.getSensorNoise(env, robot)
        # stddev = 1
        X_t = np.copy(robot._true_pose)
        zt = self.get_true_measure(X_t)
        zt += np.random.normal(mean, stddev, X_t.shape)

        self.zs.append([len(self.zs), zt])
        return zt
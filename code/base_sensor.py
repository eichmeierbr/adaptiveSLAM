import numpy as np

## TODO: Need way to store estimated feature location
## TODO: Each sensor needs an error function
## TODO: Need someway to extract features from each class that has them


class Base_sensor:
    """
    What are P_map P_est???
    """
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, dim=2, freq=0.01):
        self._local = False
        self.__P_map = np.ones_like(in_map) * P_des
        self._P_est = np.ones(dim)*P_est
        self._freq  = freq
        self._last_meas = -10
        self._dim = dim
        self._sensor = None 
        self._num_features = 0
        self.features = np.array([])

    def get_values(self):
        """
        Get the values that we're trying to optimize.
        The most likely examples are landmarks or features.
        """
        return 

    def set_values(self,feature):
        """
        Set the values we're trying to optimize. These are usually landmarks
        """
        self._num_features = len(feature)
        self.features = feature


    def error_function(self, p, z, args=[]):
        """
        Compute the error for the optimizer. The error should assume the sensor reading
        is perfect, and return the error from the true measurement with the given args
        \param p        current pose estimate
        \param z        sensor reading
        \param args     any other arguments that may be useful wrapped in a list
        """
        return z-p 


    def get_true_measure(self, p, args=[]):
        """
        Given p and some arguments, return a sensor reading. This function should be
        called in the getMeasure function (where noise is added). This function
        represents the model of the sensor and should not have any noise.
        \param p        current pose estimate
        \param args     any other arguments that may be useful wrapped in a list
        \param z        output sensor measurement (may be a list)
        """
        z = p
        return z


    def getMeasure(self, env, robot):
        """
        Retrieve simulated sensor measurement from the robot.
        Get ideal measurement from get_true_measure, then add noise.
        \param map      Map of the robot's environment
        \param robot    Robot object containing state information
        \param zt       Sensor measurement
        """

        zt = []
        (mean,stddev) = self.getSensorNoise(env, robot)
        X_t = robot._true_pose

        if self.features.size > 0:
            for feature in self.features:
                meas = self.get_true_measure(X_t, feature)
                meas += np.random.normal(mean,stddev, len(meas))
                zt.append(meas)
        else:
            meas = self.get_true_measure(X_t)
            meas += np.random.normal(mean,stddev, len(meas))
            zt.append(meas)
        return zt


    
    def getSensorNoise(self, env, robot):
        """
        Retrieve simulated sensor covariance from envorinment. For now, return zeros.
        \param env      Map of the robot's environment
        \param robot    Robot object containing state information
        """
        return env.getSensorNoise(self,robot)
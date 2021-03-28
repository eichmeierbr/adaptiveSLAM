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

    def get_values(self):
        return 

    def error_function(self):
        ## Return error for the optimizer
        return


    def getMeasure(self, env, robot):
        """
        Retrieve simulated sensor measurement from the robot. For now, return zeros.
        \param map      Map of the robot's environment
        \param robot    Robot object containing state information
        \param zt       Sensor measurement
        """

        X_t = robot._true_pose
        P_true =self.__P_map[int(X_t[0]), int(X_t[1])]
        zt = X_t + np.random.normal(P_true)
        return zt
    
    def getSensorNoise(self, env, robot):
        """
        Retrieve simulated sensor covariance from envorinment. For now, return zeros.
        \param env      Map of the robot's environment
        \param robot    Robot object containing state information
        """
        return env.getSensorNoise(self,robot)
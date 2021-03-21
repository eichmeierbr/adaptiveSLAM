import numpy as np

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

    def get_values(self):
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
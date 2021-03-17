import numpy as np

class base_sensor:
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, freq=0.01):
        self._local = True
        self.__P_map = np.ones_like(in_map) * P_des
        self._P_est = P_est
        self._freq  = freq
        self._last_meas = -10

    def getMeasure(self, map, robot):
        """
        Retrieve simulated sensor measurement from the robot. For now, return zeros.
    
        \param map      Map of the robot's environment
        \param robot    Robot object containing state information
        \param zt       Sensor measurement
        """

        zt = np.zeros(2)
        return zt
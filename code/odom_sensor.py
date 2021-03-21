# odom Sensor
from base_sensor import *
class odometry_sensor(Base_sensor):
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, dim=2, freq=0.01):
        self._local = False
        self.__P_map = np.ones_like(in_map) * P_des
        self._P_est = np.ones(dim)*P_est
        self._freq  = freq
        self._last_meas = -10
        self._dim = dim
        self._sensor = "odometry"


    def set_values(self,feature):
        self.features = feature


    def get_values(self):
        return self.features



    def getMeasure(self, env, robot):
        """
        Retrieve simulated sensor measurement from the robot. For now, return zeros.
        \param map      Map of the robot's environment
        \param robot    Robot object containing state information
        \param zt       Sensor measurement
        """
        #TODO: change to sensor values we need
        
        (mean,stddev) = self.getSensorNoise(env, robot)
        X_t_est = robot._est_pose
        u_t_est = robot._u_t
        zt = X_t_est + u_t_est + np.random.normal(mean, stddev, (2,1))
        return zt
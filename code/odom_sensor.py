# odom Sensor
from base_sensor import *
class odometry_sensor(Base_sensor):
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, dim=2, freq=0.01):
        super().__init__(local, in_map, P_est, P_des, dim, freq)
        self._sensor = "odometry"
        self._odom_func = None


    def getMeasure(self, env, robot):
        """
        Retrieve simulated sensor measurement from the robot. For now, return zeros.
        \param env      Map of the robot's environment
        \param robot    Robot object containing state information
        \param zt       Sensor measurement
        """
        #TODO: change to sensor values we need
        
        (mean,stddev) = self.getSensorNoise(env, robot)
        # stddev = 1
        X_t_est = robot._est_pose

        ut = robot._odom._true_odom

        zt = ut + np.random.normal(mean, stddev, ut.shape)
        return zt

    def get_true_measure(self, pt, args):
        pt1 = args[0]
        rob = args[1]
        return rob._odom.get_true_measure(pt1, pt)


    def error_function(self, p, z, args=[]):
        """
        Compute the error for the optimizer. The error should assume the sensor reading
        is perfect, and return the error from the true measurement with the given args
        \param p        current pose estimate
        \param z        sensor reading
        \param args     any other arguments that may be useful wrapped in a list
        """

        pt1 = args[0]
        rob = args[1]
        ut = self.get_true_measure(p, args)
        return ut - z 


    def get_nominal_path(self, zs, args=[]):
        path = []
        pose = args

        for idx, z in zs:
            pose = self._odom_func(pose, z)
            path.append(pose)

        return np.array(path)
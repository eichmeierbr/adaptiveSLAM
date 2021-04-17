# feature_sensor
from base_sensor import *


class feature_sensor(Base_sensor):
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, dim=2, freq=0.01, features=np.array([[300,200],[200,150],[350,150]])):
        super().__init__(local, in_map, P_est, P_des, dim, freq)
        self._sensor = "feature"

        self.features = features 
        self._num_features = len(features)
        self.feature_dim = features.shape[1]

        self.last_feat_pos = []

    def get_values(self):
        return self.features

    def warp2pi(self, angle_rad):
        """
        TODO: warps an angle in [-pi, pi]. Used in the update step.

        \param angle_rad Input angle in radius
        \return angle_rad_warped Warped angle to [-\pi, \pi].
        """
        pi = 3.14159
        if angle_rad>0:
            angle_rad = angle_rad%(pi*2)
        else:
            angle_rad = angle_rad%(-pi*2)
        if angle_rad>pi:
            return angle_rad-pi*2
        if angle_rad<-pi:
            return angle_rad+pi*2
        return angle_rad

    def error_function(self, p, zs, args):
        err = []
        ls = args[0]
        for l, z in zip(ls, zs):
            zt = self.get_true_measure(p, l)
            d = z - zt
            err.append(d)

        return np.array(err)


    def get_true_measure(slef, p, l):
        ## Nonlinear range and bearing
        # distance = np.sqrt((l[0]-p[0])**2+(l[1]-p[1])**2)
        # angle = np.arctan2((l[1]-p[1]),(l[0]-p[0]))
        # return distance, angle

        ## Linear dx, dy
        d = l-p
        return np.array(d)

    # def getMeasure(self, env, robot):
    #     """
    #     Retrieve simulated sensor measurement from the robot. For now, return zeros.
    #     \param map      Map of the robot's environment
    #     \param robot    Robot object containing state information
    #     \param zt       Sensor measurement
    #     """
    #     #TODO: change to sensor values we need
    #     # X_t = robot._true_pose
    #     # P_true =self.__P_map[int(X_t[0]), int(X_t[1])]
    #     # zt = X_t + np.random.normal(P_true)
    #     # return zt


    #     #chooses a random feature, finds angle and applies noise to distance based on angle, returns new location
    #     zt = []
    #     (mean,stddev) = self.getSensorNoise(env, robot)
    #     # feature = self.features[np.random.choice(range(len(self.features))),:]
    #     X_t = robot._true_pose
    #     for feature in self.features:
    #         meas = self.get_true_measure(X_t, feature)
    #         meas += np.random.normal(mean,stddev, len(meas))
    #         # X_t_est = np.array([feature[0]-np.cos(angle)*new_dist,feature[1]-np.sin(angle)*new_dist])
    #         zt.append(meas)
    #     return zt
    #     # return np.concatenate((X_t_est,X_t[2:]),axis=0)


    ###############################################################################################################
    ########            This may need to be deleted. It's a temp function

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
        # stddev = 0.1
        # mean = 0.0
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

        self.last_feat_pos = np.array(zt) + robot._est_pose
        return zt


    #############
    ##########################################################################################################
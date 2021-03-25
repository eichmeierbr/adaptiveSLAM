# feature_sensor
from base_sensor import *


class feature_sensor(Base_sensor):
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, dim=2, freq=0.01, features=np.array([[300,200],[200,150],[350,150]])):
        self._local = False
        self.__P_map = np.ones_like(in_map) * P_des
        self._P_est = np.ones(dim)*P_est
        self._freq  = freq
        self._last_meas = -10
        self._dim = dim
        self._sensor = "feature"

        self.features = features 

    def set_values(self,feature):
        self.features = feature


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


    def getMeasure(self, env, robot):
        """
        Retrieve simulated sensor measurement from the robot. For now, return zeros.
        \param map      Map of the robot's environment
        \param robot    Robot object containing state information
        \param zt       Sensor measurement
        """
        #TODO: change to sensor values we need
        # X_t = robot._true_pose
        # P_true =self.__P_map[int(X_t[0]), int(X_t[1])]
        # zt = X_t + np.random.normal(P_true)
        # return zt


        #chooses a random feature, finds angle and applies noise to distance based on angle, returns new location
        (mean,stddev) = self.getSensorNoise(env, robot)
        feature = self.features[np.random.choice(range(len(self.features))),:]
        X_t = robot._true_pose
        distance = np.sqrt((feature[0]-X_t[0])**2+(feature[1]-X_t[1])**2)
        angle = np.arctan2((feature[1]-X_t[1]),(feature[0]-X_t[0]))
        new_dist = np.random.normal(distance+mean,stddev,[1,1])[0][0]
        X_t_est = np.array([feature[0]-np.cos(angle)*new_dist,feature[1]-np.sin(angle)*new_dist])
        return np.concatenate((X_t_est,X_t[2:]),axis=0)
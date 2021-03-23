# feature_sensor
from base_sensor import *


class feature_sensor(Base_sensor):
    def __init__(self, local=True, in_map=np.zeros([1000,1000]), P_est=0.1, P_des=0.1, dim=2, freq=0.01):
        self._local = False
        self.__P_map = np.ones_like(in_map) * P_des
        self._P_est = np.ones(dim)*P_est
        self._freq  = freq
        self._last_meas = -10
        self._dim = dim
        self._sensor = "feature"

        self.features = np.array([[300,200],[200,150],[350,150]])
        k = len(self.features)
        # self.P_pre = np.block([[np.zeros((3, 3)), np.zeros((3, 2 * k))],
        #                        [np.zeros((2 * k, 3)), np.zeros((3, 3))]])

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
        X_t = robot._true_pose
        P_true =self.__P_map[int(X_t[0]), int(X_t[1])]
        zt = X_t + np.random.normal(P_true)
        return zt

        # (mean,stddev) = self.getSensorNoise(env, robot)
        # state_size = len(robot._est_pose)
        # k = len(self.features)
        # features = np.flatten(self.features)
        # X = np.concatenate((robot._true_pose.copy(),features)) #change to est for slam rather than measure
        # X_pre = X.copy()
        # # du = robot._u_t_commanded
        # # control = np.array([np.sqrt(du[0]**2+du[1]**2)[0],du[2]])
        # measure_dxdy = np.zeros(2*k,1)
        # for i in range(state_size,len(X_pre),2):
        #     measure_dxdy[i] = X_pre[i+state_size]-X_pre[0] + np.random.normal(mean,stddev,(1,1))
        #     measure_dxdy[i+1] = X_pre[i+1+state_size]-X_pre[1] + np.random.normal(mean,stddev,(1,1))
        # return measure_dxdy #returns measurement to each landmark get estimated state in slam?

        # # my hw2 if we need to send an xy estimated pose - we will need to run a mini slam to estimate pose for this sensor
        # X_pre[0:state_size] = X[0:state_size]+du
        # #find x_pre[x,y,theta] with error terms and make jacobian of that with respect to x for F, and error for L
        # # xpre[x] = px+control[0]*np.cos(X[2])+(error[x]+error[y])*np.cos(X[2])
        # # xpre[y] = py+control[0]*np.sin(X[2])+(error[x]+error[y])*np.sin(X[2])
        # # xpre[t] = pt+alpha+erroralpha
        # Ft = np.array([1,0,control[0][0]*-np.sin(X[2])[0],0,1,control[0][0]*np.cos(X[2])[0],0,0,1], dtype=np.float32 ).reshape(state_size,state_size)
        
        # Lk = np.array([control[0][0]*np.cos(X[2])[0],control[0][0]*np.cos(X[2])[0],0,control[0][0]*np.sin(X[2])[0],control[0][0]*np.sin(X[2])[0],0,0,0,1], dtype=np.float32 ).reshape(state_size,state_size)
        
        # P_pre[0:state_size,0:state_size] = Ft@P_pre[0:state_size,0:state_size]@Ft.transpose()+Lk@control_cov@Lk.transpose()
        
        # for ik in range(state_size,2*k,2):
        #     dx = X_pre[ik]-X_pre[0]
        #     dy = X_pre[ik+1]-X_pre[1]
        #     theta = X_pre[2][0]
        #     dx=dx[0]
        #     dy=dy[0]
        #     delta= np.array([dx,dy], dtype=np.float32 )
        #     q = delta.transpose()@delta
        #     sqrtq = np.sqrt(q)

        #     measure_predict = np.array([self.warp2pi(np.arctan2(dy,dx)-theta),sqrtq], dtype=np.float32 )
            
        #     Fpart1 = np.concatenate((np.eye(state_size),np.zeros((2,state_size))),axis=0)
        #     Fpart2 = np.zeros((state_size+2,ik-state_size))
        #     Fpart3 = np.concatenate((np.zeros((state_size,2)),np.eye(2)),axis=0)
        #     Fpart4 = np.zeros((state_size+2,2*k-ik+1))
        #     F = np.concatenate((Fpart1,Fpart2,Fpart3,Fpart4),axis=1)
            
        #     Hpart1 = np.array([dy,-dx,-q,-dy,dx,-sqrtq*dx,-sqrtq*dy,0,sqrtq*dx,sqrtq*dy], dtype=np.float32 ).reshape(2,state_size+2)
        #     H = 1/q*Hpart1@F
            
        #     K = P_pre @ H.transpose() @ np.linalg.inv(H @ P_pre @ H.transpose() + measure_cov )

        #     X_pre = X_pre + (K @ ( measure[ik-state_size:ik-state_size+2] - measure_predict.reshape(2,1) )).reshape(X_pre.size,1)

        #     P_pre = (np.eye(P_pre.shape[0]) - K @ H) @ P_pre
        
        # return X_pre[0:state_size]
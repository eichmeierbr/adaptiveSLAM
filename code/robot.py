import numpy as np
from odometry import *


class Robot:
    def __init__(self, init_pose=np.zeros(2), env=np.zeros([1000,1000]), odom=Diff_movement()):

        # Init pose variables
        self._true_pose = init_pose[:]
        self._est_pose  = init_pose[:]
        self._true_path = [self._true_pose]
        self._est_path  = [self._true_pose]

        # Init current ut
        self._u_t_commanded = init_pose[:]
        self._u_t = init_pose[:]

        # Init belief map
        self._belief_map = np.zeros_like(env)

        # Init motion model
        self._odom=odom



    def get_pose(self, est_pose=True):
        if est_pose:
            return self._est_pose
        else:
            return self._true_pose

    
    def get_path(self, est_path=True):
        if est_path:
            return self._est_path
        else:
            return self._true_path

    
    def move(self, ut):
        """
        Perform an odometry step.
    
        \param ut         Control input
        """
        self._u_t_commanded = ut.copy()
        ## Call odometry function to get the new poses
        self._true_pose, self._est_pose = self._odom.take_step(self._true_pose, self._est_pose, ut)
        self._u_t = self._true_pose-self._true_path[-1]
        ## Update path history
        self._true_path.append(self._true_pose)
        self._est_path.append(self._est_pose)


    


import numpy as np

class Diff_movement:

    def __init__(self, noise=0.1, dim=2):
        self._noise = np.ones(dim) * noise


    def take_step(self, pt_true, pt_est, ut):
        """
        Perform an odometry step and output both the true position and the estimated position.
    
        \param pt_true    True robot pose before control action
        \param pt_est     Estimated robot pose before control action
        \param ut         Control input
        \param pt1_true   True robot pose after control action
        \param pt1_est    Estimated robot pose after control action
        """

        pt1_true = pt_true + ut
        pt1_est  = pt_est  + ut + np.random.normal(self._noise)
        return pt1_true, pt1_est

class Abs_movement:

    def __init__(self, noise=0.1, dim=2):
        self._noise = np.ones(dim) * noise


    def take_step(self, pt_true, pt_est, ut):
        """
        Perform an odometry step and output both the true position and the estimated position.
    
        \param pt_true    True robot pose before control action
        \param pt_est     Estimated robot pose before control action
        \param ut         Control input (Desired Global Position)
        \param pt1_true   True robot pose after control action
        \param pt1_est    Estimated robot pose after control action
        """

        pt1_true = ut
        pt1_est  = ut + np.random.normal(self._noise)
        return pt1_true, pt1_est



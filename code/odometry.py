import numpy as np


class Motion_model:

    def __init__(self, noise=0.1, dim=2):
        self._u_noise = np.ones(dim) * noise # Noise for the odometry sensor, not necesserily odometry execution
        self._noisy_odom = []


    def take_step(self, pt_true, pt_est, ut):
        """
        Perform an odometry step and output both the true position and the estimated position.
    
        \param pt_true    True robot pose before control action
        \param pt_est     Estimated robot pose before control action
        \param ut         Control input
        \param pt1_true   True robot pose after control action
        \param pt1_est    Estimated robot pose after control action
        """
        self._noisy_odom = ut + np.random.normal(self._u_noise)

        pt1_true = self.odom_func(pt_true, ut)
        pt1_est  = self.odom_func(pt_est, self._noisy_odom)

        return pt1_true, pt1_est

    def get_measure(self):
        return self._noisy_odom

    def odom_func(self, pt, ut):
        """
        Function to compute the next pose based on odometry measurement
    
        \param pt         Starting pose
        \param ut         Control input
        \param pt1        Pose after control action
        """
        return pt1


class Diff_movement(Motion_model):

    def odom_func(self, pt, ut):
        """
        Add the control input to pt.
    
        \param pt         Starting pose
        \param ut         Control input
        \param pt1        Pose after control action
        """
        pt1 = pt + ut
        return pt1

class Abs_movement(Motion_model):

    def odom_func(self, pt, ut):
        """
        Set pt to the control input.
    
        \param pt         Starting pose
        \param ut         Control input
        \param pt1        Pose after control action
        """
        pt1 = ut
        return pt1




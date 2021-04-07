
import matplotlib.pyplot as plt
import numpy as np
import copy


class display_stuff:
    def __init__(self,robot):
        self.val = 0

    def display_map2(self,env, robot, dt, sensors):
        plt.imshow(env.map, cmap='Greys')
        # plt.scatter(robot._true_pose[0], robot._true_pose[1])
        # plt.scatter(robot._est_pose[0], robot._est_pose[1])
        


        true_path = np.array(robot._true_path)
        est_path = np.array(robot._est_path)
        noisy_path = np.array(robot._noisy_path)
        plt.plot(noisy_path[:,0], noisy_path[:,1],label="Noisy Path")
        plt.plot(est_path[:,0], est_path[:,1],label="Optimized Path")
        plt.plot(true_path[:,0], true_path[:,1],label="True Path")

        # for sens in sensors:
        #     sensor_vals = sens.getMeasure(env, robot)
        #     if sensor_vals is None:
        #         continue
        #     else:
        #         plt.scatter(sensor_vals[0], sensor_vals[1],s=0.6)
        
        plt.legend()
        plt.draw()
        plt.pause(dt)
        plt.clf()

import matplotlib.pyplot as plt
import numpy as np
import copy


class display_stuff:
    def __init__(self,robot):
        self.val = 0

    def plot_regions(self,env, robot, sensors):
        fade = 0.8



        stuff = np.abs(np.add(env.map,-1))
        blank_map = np.ones(np.append(np.shape(env.map),4))

        i=0
        for sensor in sensors:
            max_val = np.max(env.getSensorRegion(sensor,robot))
            temp = np.array(stuff*env.getSensorRegion(sensor,robot)/max_val)
            blank_map[:,:,i] = temp
            
            i = (i)%3
            i+=1

        blank_map[:,:,0:3] = (blank_map[:,:,0:3])
        blank_map[:,:,3] = blank_map[:,:,3]-stuff*fade

        plt.imshow(blank_map)


    def display_map2(self,env, robot, dt, sensors):
        fig1 = plt.figure(1)
        fig1.set_figheight(11)
        fig1.set_figwidth(11)

        self.plot_regions(env, robot, sensors)
        # plt.imshow(env.map, cmap='Greys')
        # plt.scatter(robot._true_pose[0], robot._true_pose[1])
        # plt.scatter(robot._est_pose[0], robot._est_pose[1])
        


        true_path = np.array(robot._true_path)
        est_path = np.array(robot._est_path)
        noisy_path = np.array(robot._noisy_path)
        plt.plot(noisy_path[:,0], noisy_path[:,1],label="Noisy Path")
        plt.plot(est_path[:,0], est_path[:,1],label="Optimized Path")
        plt.plot(true_path[:,0], true_path[:,1],label="True Path")


        ## plot noisy regions:
        # for i in sensors:
        
        # ax = fig1.add_subplot(1,1,1)
        # ax.set_facecolor('red')

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
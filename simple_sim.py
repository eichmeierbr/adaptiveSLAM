import numpy as np

import matplotlib.pyplot as plt
from base_sensor import *

def display_map(env, robot):
    plt.imshow(env, cmap='Greys')
    plt.show()


if __name__ == "__main__":
    
    dt  = 0.001
    t   = 0

    ## Initialize Map
    env = np.loadtxt('maps/empty_map.csv', delimiter=',')


    ## Initialize Robot


    ## Initialize Sensors


    ## Initailze Path


    ## For time duration
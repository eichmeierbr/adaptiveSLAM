

import numpy as np
from enviornment import *
from pathlib import Path
import random

def rolling_avg(path,num):

    for i in range(num):
        pathx = [path[0,0]]
        pathy = [path[0,1]]
        for i in range(1,len(path)-1):
            avg_x = (path[i-1,0]+path[i,0]+path[i+1,0])/3
            avg_y = (path[i-1,1]+path[i,1]+path[i+1,1])/3

            # pathx.append(path[i,0])
            # pathy.append(path[i,1])

            pathx.append(avg_x)
            pathy.append(avg_y)
        
        path = np.array([pathx,pathy]).T

    return path

def get_path(waypoints,step_size=10,smooth=False):

    ##np.arange has an error with floating point error
    #solution is to use linspace but might take more work
    #also path is returned as in as the smallest value

    temp_correction_for_error = 0.01*step_size

    pathx = [waypoints[0,0]]
    pathy = [waypoints[0,1]]

    for i in range(len(waypoints)-1):
        lx = len(pathx)
        ly = len(pathy)

        if pathx[lx-1]<waypoints[i+1,0]:
            pathx = np.append(pathx,np.arange(pathx[lx-1],waypoints[i+1,0]+temp_correction_for_error,step_size))
        else:
            pathx = np.append(pathx,np.flip(np.arange(waypoints[i+1,0],pathx[lx-1]+temp_correction_for_error,step_size)))

        if pathy[ly-1]<waypoints[i+1,1]:
            pathy = np.append(pathy,np.arange(pathy[ly-1],waypoints[i+1,1]+temp_correction_for_error,step_size))
        else:
            pathy = np.append(pathy,np.flip(np.arange(waypoints[i+1,1],pathy[ly-1]+temp_correction_for_error,step_size)))
        lx = len(pathx)
        ly = len(pathy)

        if lx>ly:
            pathy = np.append(pathy,np.ones((lx-ly,1))*pathy[ly-1])

        if ly>lx:
            pathx = np.append(pathx,np.ones((ly-lx,1))*pathx[lx-1])

    path = np.zeros((len(pathx),2))
    path[:,0] = pathx
    path[:,1] = pathy

    if smooth==True:
        path = rolling_avg(path,3)

    return path

def convert_2_local(global_path):

    local_x = [0]
    local_y = [0]

    for i in range(len(global_path)-1):
        local_x = np.append(local_x,global_path[i+1,0]-global_path[i,0])
        local_y = np.append(local_y,global_path[i+1,1]-global_path[i,1])


    local_path = np.zeros((len(local_x),2))
    local_path[:,0] = local_x
    local_path[:,1] = local_y



    return local_path


def random_features(env,num_features):
    features = np.zeros((num_features,2))
    i=0
    shape = env.__getshape__()
    while i<num_features:
        rx = random.randint(0, shape[0]-1)
        ry = random.randint(0, shape[1]-1)
        val = env.__getitem__(rx,ry)
        if env.__getitem__(rx,ry)<1:
            features[i,0]=ry
            features[i,1]=rx
            i+=1

    return features



def select_world(sensors,num,num_features=100,step=10,smooth=True):
    if num==1:
        env_map = Path("../maps/empty_map.csv")
        waypoints = np.multiply(np.array([[100, 138.0, 176.0, 191.5, 207.0, 214.0, 221.0, 232.0, 243.0, 257.0, 271.0, 285.5, 300.0, 314.5, 329.0, 343.0, 357.0, 368.0, 379.0, 385.5, 392.0, 392.0, 392.0, 385.5, 379.0, 368.0, 357.0, 339.5, 322.0, 300.0, 278.0, 260.5, 243.0, 232.0, 221.0, 214.0, 207.0, 191.5, 100],
                                          [100, 134.0, 168.0, 169.5, 171.0, 157.0, 143.0, 132.0, 121.0, 114.0, 107.0, 104.5, 102.0, 104.5, 107.0, 114.0, 121.0, 132.0, 143.0, 160.5, 178.0, 200.0, 222.0, 239.5, 257.0, 268.0, 279.0, 285.5, 292.0, 292.0, 292.0, 285.5, 279.0, 268.0, 257.0, 243.0, 229.0, 205.5, 100]]).T,1.25)
    elif num==2:
        env_map = Path("../maps/building_easy.csv")
        waypoints = np.array([[100,100,600,600,100,100],
                              [100,300,300,80 ,80 ,100]]).T
    elif num==3:
        env_map = Path("../maps/sweden_map.csv")
        waypoints = np.array([[100,100,350,350,600,600,100,100],
                              [100,300,300,200,200,80 ,80 ,100]]).T   
    elif num==4:
        env_map = Path("../maps/building.csv")
        waypoints = np.array([[100,100,240,240,400,400,400,100,100],
                              [100,200,200,300,300,350,300,300,100]]).T
    else:
        env_map = Path("../maps/empty_map.csv")
        waypoints = np.array([[100,100,500,500,350,100],
                              [100,300,300,100,100,100]]).T

    waypoints = get_path(waypoints,step_size=step,smooth=smooth)
    waypoints = convert_2_local(waypoints)
    env = Enviornment(sensors,env_map)
    features = random_features(env,num_features)
    return waypoints,env,features
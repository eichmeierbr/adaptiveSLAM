

import numpy as np

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
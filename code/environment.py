import numpy as np
class Environment():
    def __init__(self, sensors, map_name):
        self.map = np.loadtxt(map_name, delimiter=',') #grabs the map
        self.sensorNoise = dict()
        for i,sensor in enumerate(sensors):
            if not sensor._sensor == None:
                filename = map_name.parents[0] / (map_name.stem+'_'+sensor._sensor+".npy")
                self.makeSensorRegions(filename, map, (i+1)%4)
                self.sensorNoise[sensor._sensor] = np.load(filename)
    
    def getSensorNoise(self,sensor,robot):
        X_t = robot._true_pose
        res=self.sensorNoise[sensor._sensor][int(X_t[1]), int(X_t[0]),:]
        return res

    def getSensorRegion(self,sensor,robot):
        X_t = robot._true_pose
        reg=self.sensorNoise[sensor._sensor][:,:,1]
        return reg

    def makeSensorRegions(self,filename, map, quarter):
        noisy_std = 3
        good_std = 1
        new_mean = "random"

        mean,stddev = 0,1
        a=np.empty((self.map.shape[0],self.map.shape[1],2))
        a[:,:,mean] = 0
        a[:,:,stddev] = good_std

        # if (quarter == 0):
        #     a[0:self.map.shape[0]//2,     0:self.map.shape[1]//2,stddev]=100
        if (quarter == 1):
            if (new_mean == 'mgrid'):
                X,Y = np.mgrid[0:self.map.shape[0]-self.map.shape[0]//2:1,0:self.map.shape[1]//2:1]
                a[self.map.shape[0]//2:self.map.shape[0],     0:self.map.shape[1]//2,mean]=X+Y
            elif (new_mean == 'random'):
                a[self.map.shape[0]//2:self.map.shape[0],     0:self.map.shape[1]//2,mean]=np.random.randint(0, 50, (self.map.shape[0]-self.map.shape[0]//2,self.map.shape[1]//2))
            else:
                a[self.map.shape[0]//2:self.map.shape[0],     0:self.map.shape[1]//2,mean]=new_mean
            if (noisy_std == 'mgrid'):
                X,Y = np.mgrid[0:self.map.shape[0]-self.map.shape[0]//2:1,0:self.map.shape[1]//2:1]
                a[self.map.shape[0]//2:self.map.shape[0],     0:self.map.shape[1]//2,stddev]=X+Y
            elif (noisy_std == 'random'):
                a[self.map.shape[0]//2:self.map.shape[0],     0:self.map.shape[1]//2,stddev]=np.random.randint(0, 50, (self.map.shape[0]-self.map.shape[0]//2,self.map.shape[1]//2))
            else:
                a[self.map.shape[0]//2:self.map.shape[0],     0:self.map.shape[1]//2,stddev]=noisy_std
        if (quarter == 2):
            if (new_mean == 'mgrid'):
                X,Y = np.mgrid[0:self.map.shape[0]//2:1,0:self.map.shape[1]-self.map.shape[1]//2:1]
                a[0:self.map.shape[0]//2,     self.map.shape[1]//2:self.map.shape[1],mean]=X+Y
            elif (new_mean == 'random'):
                a[0:self.map.shape[0]//2,     self.map.shape[1]//2:self.map.shape[1],mean]=np.random.randint(0, 50, (self.map.shape[0]//2,self.map.shape[1]-self.map.shape[1]//2))
            else:
                a[0:self.map.shape[0]//2,     self.map.shape[1]//2:self.map.shape[1],mean]=new_mean
            if (noisy_std == 'mgrid'):
                X,Y = np.mgrid[0:self.map.shape[0]//2:1,0:self.map.shape[1]-self.map.shape[1]//2:1]
                a[0:self.map.shape[0]//2,     self.map.shape[1]//2:self.map.shape[1],stddev]=X+Y
            elif (noisy_std == 'random'):
                a[0:self.map.shape[0]//2,     self.map.shape[1]//2:self.map.shape[1],stddev]=np.random.randint(0, 50, (self.map.shape[0]//2,self.map.shape[1]-self.map.shape[1]//2))
            else:
                a[0:self.map.shape[0]//2,     self.map.shape[1]//2:self.map.shape[1],stddev]=noisy_std
        if (quarter == 3):
            if (new_mean == 'mgrid'):
                X,Y = np.mgrid[0:self.map.shape[0]-self.map.shape[0]//2:1,0:self.map.shape[1]-self.map.shape[1]//2:1]
                a[self.map.shape[0]//2:self.map.shape[0],     self.map.shape[1]//2:self.map.shape[1],mean]=X+Y
            elif (new_mean == 'random'):
                a[self.map.shape[0]//2:self.map.shape[0],     self.map.shape[1]//2:self.map.shape[1],mean]=np.random.randint(0, 50, (self.map.shape[0]-self.map.shape[0]//2,self.map.shape[1]-self.map.shape[1]//2))
            else:
                a[self.map.shape[0]//2:self.map.shape[0],     self.map.shape[1]//2:self.map.shape[1],mean]=new_mean
            if (noisy_std == 'mgrid'):
                X,Y = np.mgrid[0:self.map.shape[0]-self.map.shape[0]//2:1,0:self.map.shape[1]-self.map.shape[1]//2:1]
                a[self.map.shape[0]//2:self.map.shape[0],     self.map.shape[1]//2:self.map.shape[1],stddev]=X+Y
            elif (noisy_std == 'random'):
                a[self.map.shape[0]//2:self.map.shape[0],     self.map.shape[1]//2:self.map.shape[1],stddev]=np.random.randint(0, 50, (self.map.shape[0]-self.map.shape[0]//2,self.map.shape[1]-self.map.shape[1]//2))
            else:
                a[self.map.shape[0]//2:self.map.shape[0],     self.map.shape[1]//2:self.map.shape[1],stddev]=noisy_std
        np.save(filename,a)
    
    def __getitem__(self,index):
        return self.map[index]
    
    def __getitem__(self,x,y):
        return self.map[x,y]

    def __getshape__(self):
        return np.shape(self.map)
import numpy as np
class Enviornment():
    def __init__(self, sensors, map_name):
        self.map = np.loadtxt(map_name, delimiter=',') #grabs the map
        self.sensorNoise = dict()
        for sensor in sensors:
            if not sensor._sensor == None:
                print('need sensor region file')
                filename = map_name#+'_'+sensor._sensor
                self.sensorNoise[sensor._sensor] = np.loadtxt(filename, delimiter=',')
    
    def getSensorNoise(self,sensor,robot):
        X_t = robot._true_pose
        res=self.sensorNoise[sensor._sensor][int(X_t[0]), int(X_t[1])]
        return res
    
    def __getitem__(self,index):
        return self.map[index]
print('deprecated. please use environment.py')
# import numpy as np
# class Enviornment():
#     def __init__(self, sensors, map_name):
#         self.map = np.loadtxt(map_name, delimiter=',') #grabs the map
#         self.sensorNoise = dict()
#         for i,sensor in enumerate(sensors):
#             if not sensor._sensor == None:
#                 filename = map_name.parents[0] / (map_name.stem+'_'+sensor._sensor+".npy")
#                 self.makeSensorRegions(filename, map, i)
#                 self.sensorNoise[sensor._sensor] = np.load(filename)
    
#     def getSensorNoise(self,sensor,robot):
#         X_t = robot._true_pose
#         res=self.sensorNoise[sensor._sensor][int(X_t[1]), int(X_t[0]),:]
#         return res

#     def makeSensorRegions(self,filename, map, quarter):
#         mean,stddev = 0,1
#         a=np.empty((self.map.shape[0],self.map.shape[1],2))
#         a[:,:,mean] = 0
#         a[:,:,stddev] = 1
#         if (quarter == 1):
#             a[0:self.map.shape[0]//2,     0:self.map.shape[1]//2,stddev]=100
#         if (quarter == 2):
#             a[self.map.shape[0]//2:self.map.shape[0],     0:self.map.shape[1]//2,stddev]=100
#         if (quarter == 3):
#             a[0:self.map.shape[0]//2,     self.map.shape[1]//2:self.map.shape[1],stddev]=100
#         if (quarter == 4):
#             a[self.map.shape[0]//2:self.map.shape[0],     self.map.shape[1]//2:self.map.shape[1],stddev]=100
#         np.save(filename,a)
    
#     def __getitem__(self,index):
#         return self.map[index]
    
#     def __getitem__(self,x,y):
#         return self.map[x,y]

#     def __getshape__(self):
#         return np.shape(self.map)
import numpy as np
from scipy.optimize import least_squares
from sklearn.neural_network import MLPRegressor

def error_func(x, mat):
    error = []
    for i in range(mat.shape[0]):
        for j in range(i,mat.shape[1]):
            err = x[i]*x[j]
            error.append(mat[i,j]-err)
    return error
    

num = 10

std_devs = [1, 2, 7, 20]

est_paths = [] 
for stdev in std_devs:
    est_paths.append(np.random.normal(scale=stdev, size=num ))
est_paths = np.array(est_paths)

s = np.zeros([est_paths.shape[0], est_paths.shape[0]])

for i in range(est_paths.shape[0]):
    for j in range(est_paths.shape[0]):
        if i == j:
            continue
        err = est_paths[i] - est_paths[j]
        s[i,j] = np.std(err)
        b=3

noms = np.average(s, axis=0)
vals = least_squares(error_func, noms, args=([s])).x
vals /= np.min(vals)
vals = vals**4
vals = np.exp(vals)
a=3
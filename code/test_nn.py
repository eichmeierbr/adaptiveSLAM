import numpy as np
from scipy.optimize import least_squares
from sklearn.neural_network import MLPRegressor

# np.random.seed(0)


def getCovarianceMat(est_paths):
    s = np.zeros([est_paths.shape[0], est_paths.shape[0]])

    for i in range(est_paths.shape[0]):
        for j in range(est_paths.shape[0]):
            if i == j:
                continue
            err = est_paths[i] - est_paths[j]
            s[i,j] = np.std(err)

    return s

def make_example(std_devs, len_path):
    est_paths = [] 
    for stdev in std_devs:
        est_paths.append(np.random.normal(scale=stdev, size=len_path ))

    est_paths = np.array(est_paths)
    s = getCovarianceMat(est_paths)
    
    idxs = np.sort(np.unique(s, return_index=True)[1])[1:]

    return s.flatten()[idxs] 


def make_dataset(num_sensors, len_path, num_samples, min_std=1, max_std=20):
    
    xs = []
    ys = []

    for _ in range(num_samples):
        std_devs = np.random.uniform(low=min_std, high=max_std, size=num_sensors)
        
        xs.append( make_example(std_devs, len_path) )
        ys.append( std_devs)

    return np.array(xs), np.array(ys)


def split_dataset(xs, ys, train_ratio):
    split = int(len(xs)*train_ratio)
    train_x, test_x = np.split(xs, [split])
    train_y, test_y = np.split(ys, [split])
    return train_x, train_y, test_x, test_y


num_sensors = 4
len_path = 10
num_samples = 10000
train_ratio = 0.7

xs, ys = make_dataset(num_sensors, len_path, num_samples)
train_x, train_y, test_x, test_y = split_dataset(xs, ys, train_ratio)

model = MLPRegressor(hidden_layer_sizes=(30,30)).fit(train_x, train_y)


print('Label1:   ',test_y[0])
print('Predict1: ', model.predict([test_x[0]]))


test_arr = [1,2,1,7]
print('\nLabel2:   ', test_arr)
print('Predict2:   ', model.predict([make_example(test_arr, len_path)]))

print(model.score(test_x, test_y))


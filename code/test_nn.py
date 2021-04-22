import numpy as np
from scipy.optimize import least_squares
from sklearn.neural_network import MLPRegressor, MLPClassifier
import joblib

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


def make_dataset(num_sensors, len_path, num_samples, min_std=0.0, max_std=50.0):
    
    xs = []
    ys = []

    for _ in range(num_samples):
        std_devs = np.random.uniform(low=min_std, high=max_std, size=num_sensors)
        fak_devs = std_devs + np.random.normal(scale=3, size=len(std_devs))
        
        xs.append( np.hstack((fak_devs,make_example(std_devs, len_path))) )
        # ys.append( (std_devs > 15)*1)
        ys.append( std_devs )

    return np.array(xs), np.array(ys)


def split_dataset(xs, ys, train_ratio):
    split = int(len(xs)*train_ratio)
    train_x, test_x = np.split(xs, [split])
    train_y, test_y = np.split(ys, [split])
    return train_x, train_y, test_x, test_y


num_sensors = 3
len_path = 10
num_samples = 3000000
train_ratio = 0.7

xs, ys = make_dataset(num_sensors, len_path, num_samples)
train_x, train_y, test_x, test_y = split_dataset(xs, ys, train_ratio)

# model = MLPRegressor(hidden_layer_sizes=(30,30)).fit(train_x, train_y)
# model = MLPClassifier(hidden_layer_sizes=(30,30)).fit(train_x, train_y)
model = joblib.load('first_model2.joblib')

print('Label1:   ',test_y[0])
print('Predict1: ', model.predict([test_x[0]]))


test_arr = [1,20,13]
fake_arr = [14, 2, 3]
fake_arrs= []
fake_arrs.append(fake_arr)

print('\nLabel2:   ', test_arr)
print('Seed:   ', fake_arr)

for i in range(1000):
    fake_arr = model.predict([np.hstack((fake_arr, make_example(test_arr, len_path)))]).flatten()
    fake_arrs.append(fake_arr)

fake_arrs = np.array(fake_arrs)
pred = np.median(fake_arrs[:-100], axis=0 )
print('Final Pred: ', pred)

print(model.score(test_x, test_y))

joblib.dump(model,'first_model.joblib')
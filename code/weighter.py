import numpy as np
from scipy.optimize import least_squares
from sklearn.neural_network import MLPRegressor, MLPClassifier
from joblib import load, dump


class weighter():
    def __init__(self, model_file='first_model.joblib'):
        self.model = load(model_file)
        self.num_guesses = 200
        self.max_stddev = 50.0
        self.min_stddev = .1


    def get_nominal_paths(self, sensors, zs, anchor):
        """
        Compute the nominal path for each sensor. Assuming that sensor is correct, where was the robot?
        \param sensors      Sensors used to gather zs.
        \param zs           sensor readings
        \param anchor       Starting pose of the optimization
        \param est_paths    Estimated robot paths
        """
        est_paths = []

        ## Separate out sensor measurements by sensor
        zs_sensor = {}
        for zt in zs:
            for z in zt[1]:
                if z[0] in zs_sensor:
                    zs_sensor[z[0]].append([zt[0], z[1]])
                else:
                    zs_sensor[z[0]] = [[zt[0], z[1]]]

        ## Estimate a path for each sensor.
        ## NOTE: This assumes a datapoint is given at each time stamp.
        for i, sense in enumerate(sensors):
            z = zs_sensor[i]
            est_paths.append(sense.get_nominal_path(z, args=anchor))
        
        return np.array(est_paths)


    def getCovarianceMat(self, est_paths):
        """
        Convert estimated paths into relative "pseudo" standard deviations
        \param est_paths   Estimated robot paths, one path for each sensor
        \param s           Pseudo-standard deviations
        """
        s = []

        ## Compute relative path deviation between each estimated path
        for i in range(est_paths.shape[0]):
            for j in range(i, est_paths.shape[0]):
                if i == j:
                    continue
                ## Find the standard deviation from one path to another
                err = est_paths[i] - est_paths[j]
                s.append(np.std(err))

        return np.array(s) 


    def predict_stddevs(self, s, sensors, prior=None):
        """
        Extract sensor standard devations from the pseudo deviations
        \param s            Pseudo-standard deviations
        \param sensors      Sensors used to gather zs.
        \param prior        Beginning guess for the standard deviations
        \param pred         Predicted standard deviations
        """

        # If there is no prior, assume all sensors are equally weighted
        ## TODO: Potentially have expected standard deviations here.
        if prior == None:
            prior = np.ones(len(sensors))
        
        # Create vector of predictions for the iterative solver
        preds  = [prior]

        ## Sample n predictions. Use the last prediction to seed the next prediction
        for _ in range(self.num_guesses):
            features = np.hstack((preds[-1], s)).reshape([1,-1])
            preds.append(self.model.predict(features).flatten())

        # Return the median prediction of the last n/10 guesses
        preds = np.array(preds)
        pred = np.median(preds[:-int(self.num_guesses/10)], axis=0 )
        return pred


    def stddevs_to_weights(self, stddevs, exponent=1.0):
        """
        Convert standard deviations to weights. For now, reciprocal
        \param stddevs      Predicted standard deviations
        \param weights      Sensor weights output to the solver
        """
        # Bound weights between min and max values
        stddevs[stddevs > self.max_stddev] = self.max_stddev
        stddevs[stddevs < self.min_stddev] = self.min_stddev

        # Return reciporcal, or another funct if needed
        tiny_funct = 1/stddevs
        return tiny_funct**exponent



    def get_weights(self, sensors, zs, anchor, exponent=1.0):
        """
        Find regionally adaptive sensor weights for the given sensor data.
        \param sensors      Sensors used to gather zs.
        \param zs           sensor readings
        \param anchor       Starting pose of the optimization
        \param weights      Sensor weights output to the solver

        """
        est_paths = self.get_nominal_paths(sensors, zs, anchor)

        s = self.getCovarianceMat(est_paths)
        stddevs = self.predict_stddevs(s, sensors)

        weights = self.stddevs_to_weights(stddevs, exponent=exponent)
        w = np.array(len(zs)*[weights])
        return w
        # return np.ones(len(sensors))


    def get_all_weights(self, sensors, zs, poses, last_opt, opt_freq):

        if len(zs) < opt_freq*2:
            return self.get_weights(sensors, zs[:opt_freq], poses[0])

        ## Do first opt_freq/2 weights
        first_opts = self.get_weights(sensors, zs[last_opt:last_opt+opt_freq], poses[last_opt], exponent=2.5)

        ## Do last opt_freq/2 weights
        last_opts = self.get_weights(sensors, zs[-opt_freq:], poses[-opt_freq-1], exponent=2.5)

        stacks = np.vstack((first_opts, last_opts))


        ## Do all in between weights
        for i in range(last_opt, last_opt+opt_freq):
            ws = self.get_weights(sensors, zs[i:i+opt_freq], poses[i], exponent=2.5)
            stacks[int(i-last_opt+opt_freq/2),:] = ws[0]

        ## Smooth Transition Weights
        # for i in range(stacks.shape[1]):
            # stacks[:,i] = np.linspace(stacks[0,i], stacks[-1,i], len(stacks))

        return stacks
        # return np.vstack((first_opts, last_opts))

import numpy as np
import matplotlib.pyplot as plt
from simple_sim import run_sim


def compute_errors(robot, sensors):
    opted = np.linalg.norm(robot._true_path - robot._est_path, axis=1)
    noisy = np.linalg.norm(robot._true_path - robot._noisy_path, axis=1)

    sensor_errs = []
    for i in range(len(sensors)):
        sense_path = np.array(sensors[i].get_nominal_path(sensors[i].zs, [robot._est_path[0]])).reshape([-1,2])
        sens_err = np.linalg.norm(robot._true_path[1:] - sense_path, axis=1)

        sensor_errs.append(sens_err)
    
    return opted, noisy, sensor_errs


def plot_errors(opt, noisy, sens, sensors, plot_sensors=True):
    N = 5
    opt = np.convolve(opt, np.ones(N)/N, mode='valid')
    noisy = np.convolve(noisy, np.ones(N)/N, mode='valid')

    plt.plot(np.arange(len(opt)), opt, label='Opt')
    plt.plot(np.arange(len(noisy)), noisy, label="Baseline")

    colors = {0:'r', 1:'g', 2:'m'}
    if plot_sensors:
        for i in range(len(sens)):
            sens[i] = np.convolve(sens[i], np.ones(N)/N, mode='valid')
            plt.plot(np.arange(len(sens[i])), sens[i], colors[i], label=sensors[i]._sensor)
    
    plt.yscale('log')
    plt.xlabel('Time')
    plt.ylabel('Offtrack Error')
    plt.legend()
    plt.show()


def find_average_offtrack_error(num_runs=5):

    opt_errors = [[],[], []]
    noisy_errors = []
    sens_errors = [[],[],[]]

    for j in range(num_runs):
        seed = np.random.randint(2**32)

        ## Perform Sim and store opt_errors
        for i in range(-1,2):
            if i == 2:
                meth = -1
            else: meth = i
            robot, sensors = run_sim(plot=False, opt_method=meth, seed=seed)

            opt_error, noisy_error, sensor_errors = compute_errors(robot, sensors)
            opt_errors[i].append(np.average(opt_error))

        ## Store Sensor errors
        for i in range(3):
            sens_errors[i].append(np.average(sensor_errors[i]))

        ## Store Noisy Errors
        noisy_errors.append(np.average(noisy_error))

        print('Run %i/%i' %(j+1, num_runs))

    ## Print out opt_error results
    for i in range(-1,2):
        if i == 2:
            meth = -1
        else: meth = i
        av = np.average(opt_errors[i])
        std = np.std(opt_errors[i])
        print('Method %i Average Error: %.2f +- %.2f' %(meth, av, std))

    ## Print out sensor error results
    for i in range(3):
        av = np.average(sens_errors[i])
        std = np.std(sens_errors[i])
        print('%s sensor Average Error: %.2f +- %.2f' %(sensors[i]._sensor, av, std))

    ## Print out noisy error results
    av = np.average(noisy_errors)
    std = np.std(noisy_errors)
    print('Noisy Average Error: %.2f +- %.2f' %(av, std))


if __name__ == "__main__":
    find_average_offtrack_error(num_runs=20)

    # robot, sensors = run_sim(plot=False, opt_method=0)
    # opt_error, noisy_error, sensor_errors = compute_errors(robot, sensors)
    # plot_errors(opt_error, noisy_error, sensor_errors, sensors)
import numpy as np
import os

import rospkg


def n2s(x, precision=2):
    """
    converts numpy array to string.

    Arguments
    ---------
    `x`: `np.array`
        The numpy array to convert to a string.

    `precision`: `int`
        The precision desired on each entry in the array.

    """
    return np.array2string(x, precision=precision, separator=',', suppress_small=True)


def get_poses_list_file(filename):
    """
    Gets poses list from `filename`, that is located in the
    path to `ros_robotic_skin`, in the `data` folder.

    Arguments
    ----------
    `filename`: `str` The txt filename of where the positions are stored.

    Returns
    ----------
    returns: `poses_list`: `list`
        Consists of a list of `p` entries
        for each pose, where each of these entries contains a list of
        `j` joint positions, an empty lits, and a string
        that consists of `Pose_{p+1}`, where p is the pose number.
    """
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    # load the matrix, where the shape is n poses by j joints
    poses_mat = np.loadtxt(os.path.join(ros_robotic_skin_path, 'config', filename))
    poses_list = []
    if len(poses_mat.shape) == 1:
        # only one pose, but need to add on a dimension
        poses_mat = np.expand_dims(poses_mat, axis=0)
    for idx, pose in enumerate(poses_mat):
        poses_list.append([list(pose), [], 'Pose_{}'.format(idx+1)])
    return poses_list

def hampel_filter_forloop(input_series, window_size, n_sigmas=3):
    """
    Implementation of Hampel Filter for outlier detection.

    Arguments
    ----------
    `input_series`: `np.array`
        The input data to use for outlier detection.

    `window_size`: `int`
        The sliding window size to use for the filter on
        `input_series`.

    `n_sigmas`: `int`
        The number of standard deviations to determine
        what data points are outliers.
    """
    n = len(input_series)
    new_series = input_series.copy()
    k = 1.4826  # scale factor for Gaussian distribution
    indices = []
    # possibly use np.nanmedian
    for i in range((window_size), (n - window_size)):
        x0 = np.median(input_series[(i - window_size):(i + window_size)])
        S0 = k * np.median(np.abs(input_series[(i - window_size):(i + window_size)] - x0))
        if (np.abs(input_series[i] - x0) > n_sigmas * S0):
            new_series[i] = x0
            indices.append(i)
    return new_series, indices


def moving_avg_low_pass_filter(data, window_size):
    """
    Implementation of a standard moving average low pass filter.

    Arguments
    ---------
    `data`: `np.array`
        data to be filtered.

    `window_size`: `int`
        Window size from which the average is calculated
        for each point

    """
    n = len(data)
    new_data = data.copy()

    for i in range(0, (n - window_size)):
        avg = np.mean(data[i: i + window_size])
        new_data[i] = avg
    return new_data


def low_pass_filter(data, samp_freq, cutoff_freq=15.):
    """
    Implementation of the standard pass filter,
    also known as a exponential moving average filter.

    Arguments
    ---------
    `data`:
        data to be filtered.
    `samp_freq`:
        sampling frequency of the data
    `cutoff_freq`:
        cutoff frequency; that is, data that is > = `cutoff_freq` will
        be attentuated.
    """
    # need to cut cutoff_freq in half because we apply two filters.
    half_cutoff_freq = cutoff_freq * 0.5
    n = len(data)
    # smoother data when alpha is lower
    tau = 1 / (2 * np.pi * half_cutoff_freq)
    dt = 1 / samp_freq
    alpha = dt / (dt + tau)
    new_data = data.copy()

    for i in range(1, n):
        new_data[i] = ((1 - alpha) * new_data[i-1]) + (alpha * data[i])
    reversed_data = new_data[::-1]

    for i in range(1, n):
        reversed_data[i] = ((1 - alpha) * reversed_data[i-1]) + (alpha * reversed_data[i])

    return reversed_data[::-1]


"""
utils xacro file
"""

import numpy as np
import os
import xacro
import rospkg
import rospy
import pickle


def get_imu_names_and_topics(xacro_name='panda_arm_hand.urdf.xacro',
                             directory='robots'):
    """
    parses the xacro file containing the imus, and
    returns a list of the imu names (each name has the link it's on
    also included in the string), as well as a list of the imu topics.

    Arguments
    ---------
    `xacro_name`: the name of the xacro file.

    `directory`: the directory, relative to the package path,
    from which the xacro file is parsed.

    Returns
    -------
    `imu_names`: The list of imu names, an example is `imu_link_0_panda_link1`,
    which indicates that the imu 0 is on link 1 of the model.

    `imu_topics`: The topics for each imu (in sorted order). For example,
    `rostopic echo /imu_data0` which echo the contents of the published IMU
    messages in ROS
    """
    # get all topics, sorted
    all_topics = sorted(rospy.get_published_topics())
    # get imu topics
    num_imu_topics = len([topic for topic in all_topics if 'imu_data' in topic[0]])

    imu_names = []
    imu_topics = []
    # imu mappings from imu to imu name and link it's connected to.
    imu_mappings = {}
    # get connected links for imus, in order by id.
    connected_links = get_joint_names_from_imus(xacro_name, directory)
    if num_imu_topics != len(connected_links):
        raise ValueError("Error in amount of imu topics.")

    for i in range(num_imu_topics):
        # separate information of connected link and imu_link.
        imu_string = 'imu_link{}'.format(i)
        imu_mappings[imu_string] = connected_links[i]
        imu_names.append(imu_string)
        imu_topics.append('imu_data{}'.format(i))
    # save pickle file of imu mappings that we can use later.
    with open('imu_mappings.pickle', 'wb') as handle:
        pickle.dump(imu_mappings, handle, protocol=pickle.HIGHEST_PROTOCOL)
    return imu_names, imu_topics


def get_joint_names_from_imus(filename, directory='robots'):
    """
    in order for later optimization,
    the joint the imu is connected to
    needs to be known.

    Arguments
    -----------
    `filename`: `str`: the filename of the xacro file

    `directory`: `str`: the directory relative to the package path
    from which the xacro file is parsed.

    Returns
    -----------
    The list of links the imus are in (in order by imu id)

    """
    # get full path for xacro file.
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    full_xacro_path = os.path.join(ros_robotic_skin_path, directory, filename)

    # parse xacro file, which returns xml document.
    document = xacro.parse(None, full_xacro_path)
    imu_elements = document.getElementsByTagName('xacro:imu')
    # make sure to get ids of imus in order.
    links = []
    sorted_imu_elements = sorted(imu_elements, key=lambda x: x.getAttribute('imu_id'))

    # get imu information.
    for imu_element in sorted_imu_elements:
        # get the link the imu is connected to.
        link_string = str(imu_element.getAttribute('connected_to'))
        links.append(link_string)

    return links


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


def reject_outliers(data, m=1):
    """
    Rejects outliers in a dataset.

    Arguments
    ----------
    `data`: `np.array`
        The data.

    `m`: `int`
        The amount of standard deviations from
        the mean which is considered an outlier.

    Returns
    ----------
    returns: None
    """
    is_in_std = np.absolute(data - np.mean(data, axis=0)) < m * np.std(data, axis=0)
    indices = np.where(is_in_std)
    return data[indices], indices

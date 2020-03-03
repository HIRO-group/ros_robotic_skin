import numpy as np
import os

import rospkg

def get_poses_list_file(filename):
    """
    Gets poses list from `filename`, that is located in the 
    path to `ros_robotic_skin`, in the `data` folder.

    Arguments
    ----------
    `filename`: `str` 
        The txt filename of where the positions are stored.    

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
    poses_mat = np.loadtxt(os.path.join(ros_robotic_skin_path, 'data', filename))
    poses_list = []
    for idx,pose in enumerate(poses_mat):
        poses_list.append([list(pose), [], 'Pose_{}'.format(idx+1)])
    return poses_list
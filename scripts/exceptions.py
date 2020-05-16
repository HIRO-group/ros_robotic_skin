"""
Exceptions definitions for ROS Robotic Skin.
"""


class InvalidNumJointException(Exception):
    """
    Exception raised when the len of the provided argument
    doesn't match the number of joints in a robotic arm.
    """
    pass


class InvalidTrajectoryCommandException(Exception):
    """
    Exception raised when the shape of a trajectory command
    provided is incorrect.
    """
    pass

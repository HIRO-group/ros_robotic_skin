import rospy
from scripts.exceptions import VariableNotInitializedException  # noqa:E402


class StopWatch():
    def __init__(self):
        self.start_time = None

    def start(self):
        self.start_time = rospy.get_rostime().to_sec()

    def get_elapsed_time(self):
        if not self.is_started():
            raise VariableNotInitializedException("Varible 'start_time' is not initialized")

        return rospy.get_rostime().to_sec() - self.start_time

    def restart(self):
        self.start_time = rospy.get_rostime().to_sec()

    def stop(self):
        self.start_time = None

    def is_started(self):
        return self.start_time

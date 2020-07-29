import rospy
from scripts.exceptions import VariableNotInitializedException  # noqa:E402


class StopWatch():
    def __init__(self):
        self.start_time = None
        self.timer = None

    def start(self):
        self.start_time = rospy.get_rostime().to_sec()

    def get_elapsed_time(self):
        if not self.is_started():
            raise VariableNotInitializedException("Varible 'start_time' is not initialized")

        return rospy.get_rostime().to_sec() - self.start_time

    def set_timer(self, timer):
        self.timer = timer

    def is_ended(self):
        if not self.is_started:
            raise ValueError('Not Started')
        if not self.is_timer_set:
            raise ValueError('Timer not Set')

        if self.get_elapsed_time() > self.timer:
            return True
        return False

    def is_timer_set(self):
        if self.timer:
            return True
        return False

    def restart(self):
        self.start_time = rospy.get_rostime().to_sec()

    def stop(self):
        self.start_time = None

    def is_started(self):
        if self.start_time:
            return True

        return False

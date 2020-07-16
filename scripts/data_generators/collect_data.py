sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts import utils  # noqa: E402
from scripts.data_generators.stopwatch import StopWatch  # noqa: E402
from scripts.controllers.RobotController import PandaController# noqa: E402


class DataCollector:
    def __init__(self, controller, poses_list, filepath='data/collected_data', is_sim=True):
        pass

    def collect_data(self):
        pass

    def save(self, save=True, verbose=False, clean=False):
        pass


if __name__ == '__main__':
    rospy.init_node('data_collection')

    controller = PandaController(is_sim=IS_SIM)
    filename = 'panda_positions.txt'

    poses_list = utils.get_poses_list_file(filename)
    filepath = '_'.join(['data/collected_data', robot])

    data_collector = DataCollector(controller, poses_list, filepath, IS_SIM)
    data_collector.collect_data()
    data_collector.save(save=True, verbose=False, clean=False)

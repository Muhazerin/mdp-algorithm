from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from constants import ImgRecogAlgoStatus, Bearing
from simExplFastPath import a_star_search, gen_move_cmd


def find_nearest_obstacle(list_of_coordinates, current_robot_center):
    min_index = 0
    min_distance = (abs(current_robot_center[0] - list_of_coordinates[0]['robot_pos'][0]) * 10) + \
                   (abs(current_robot_center[1] - list_of_coordinates[0]['robot_pos'][1]) * 10)
    for i in range(1, len(list_of_coordinates)):
        distance_to_obstacle = (abs(current_robot_center[0] - list_of_coordinates[i]['robot_pos'][0]) * 10) + \
                               (abs(current_robot_center[1] - list_of_coordinates[i]['robot_pos'][1]) * 10)
        if distance_to_obstacle < min_distance:
            min_index = i
            min_distance = distance_to_obstacle
    return list_of_coordinates[min_index]


def can_calibrate(robot_bearing, obstacle_map, all_corners):
    if robot_bearing == Bearing.NORTH:
        coordinate = all_corners[2]
        if coordinate[0] < 0:  # if the col is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1] + 2][coordinate[0]] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False
    elif robot_bearing == Bearing.EAST:
        coordinate = all_corners[0]
        if coordinate[1] > 19:  # if the row is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1]][coordinate[0] + 2] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False
    elif robot_bearing == Bearing.SOUTH:
        coordinate = all_corners[1]
        if coordinate[0] > 14:  # if the col is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1] - 2][coordinate[0]] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False
    else:
        coordinate = all_corners[3]
        if coordinate[1] < 0:  # if the row is out of bound
            return True
        elif obstacle_map[coordinate[1]][coordinate[0]] == 1 and \
                obstacle_map[coordinate[1]][coordinate[0] - 2] == 1:
            # if there are obstacle on top and bottom left
            return True
        else:
            return False


def pack_msg(message):
    return 'EC|' + message


class ActlImgRecogAlgo(QObject):
    finished = pyqtSignal()
    signalSendMsg = pyqtSignal(str)
    # signalTakePic = pyqtSignal()

    def __init__(self):
        super(ActlImgRecogAlgo, self).__init__()
        self.__stop = False
        self.__algo_status = ImgRecogAlgoStatus.SEEK_GOAL
        self.__robot_just_turned_left = False
        self.__front_left_dict = dict()
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__no_of_left_rotation = 0
        self.__img_id = set()
        self.__pic_taken = False
        self.__obstacle_left_hug_map = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]

        # [imgId, freq, robotBearing]
        self.__imgRecMap = [
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]]
        ]

    def reset_map(self):
        self.__obstacle_left_hug_map = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]

        # Testing against arena 4
        self.__imgRecMap = [
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]],
            [[-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None], [-1, 0, None],
             [-1, 0, None]]
        ]

    # check the top, bottom, left, right of the r,c if the robot has left hug the obstacle
    # robot pos is in x,y coordinate
    def check_surrounding_obstacle(self, r, c, explored_map, obstacle_map):
        list_of_coordinates = list()
        coordinates = [[r + 1, c],
                       [r - 1, c],
                       [r, c - 1],
                       [r, c + 1]]
        for coordinate in coordinates:
            row = coordinate[0]
            col = coordinate[1]
            if 0 <= row <= 19 and 0 <= col <= 14:
                if explored_map[row][col] == 1 and obstacle_map[row][col] == 0 and \
                        self.__obstacle_left_hug_map[row][col] == 0:
                    if coordinate == coordinates[0]:
                        list_of_coordinates.append({
                            'robot_pos': [col + 1, row + 2],
                            'bearing': Bearing.WEST
                        })
                    elif coordinate == coordinates[1]:
                        list_of_coordinates.append({
                            'robot_pos': [col + 1, row],
                            'bearing': Bearing.EAST
                        })
                    elif coordinate == coordinates[2]:
                        list_of_coordinates.append({
                            'robot_pos': [col, row + 1],
                            'bearing': Bearing.SOUTH
                        })
                    else:
                        list_of_coordinates.append({
                            'robot_pos': [col + 2, row + 1],
                            'bearing': Bearing.NORTH
                        })
        return list_of_coordinates

    def obstacle_without_left_hug(self, explored_map, obstacle_map):
        list_of_coordinates = list()
        for row in range(0, len(explored_map)):
            for col in range(0, len(explored_map[row])):
                if explored_map[row][col] == 1 and obstacle_map[row][col] == 1:
                    temp = self.check_surrounding_obstacle(row, col, explored_map, obstacle_map)
                    if temp:
                        list_of_coordinates = list_of_coordinates + temp
        return list_of_coordinates

    # Check if there is obstacle on left of robot
    def is_left_obstacle(self, robot_bearing, all_corners, obstacle_map):
        left_obstacle = False
        if robot_bearing == Bearing.NORTH:
            coordinate = all_corners[2]
            if coordinate[0] >= 0:
                for row in range(coordinate[1], coordinate[1] + 3):
                    if obstacle_map[row][coordinate[0]] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[row][coordinate[0] + 1] = 1
        elif robot_bearing == Bearing.EAST:
            coordinate = all_corners[0]
            if coordinate[1] <= 19:
                for col in range(coordinate[0], coordinate[0] + 3):
                    if obstacle_map[coordinate[1]][col] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[coordinate[1] - 1][col] = 1
        elif robot_bearing == Bearing.SOUTH:
            coordinate = all_corners[1]
            if coordinate[0] <= 14:
                for row in range(coordinate[1], coordinate[1] - 3, -1):
                    if obstacle_map[row][coordinate[0]] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[row][coordinate[0] - 1] = 1
        else:
            coordinate = all_corners[3]
            if coordinate[1] >= 0:
                for col in range(coordinate[0], coordinate[0] - 3, -1):
                    if obstacle_map[coordinate[1]][col] == 1:
                        left_obstacle = True
                        self.__obstacle_left_hug_map[coordinate[1] + 1][col] = 1
        return left_obstacle

    def mark_img(self, robot_bearing, coordinate, obstacle_map, image_id):
        try:
            if robot_bearing == Bearing.NORTH:
                for row in range(coordinate[1], coordinate[1] + 3):
                    if 0 <= row <= 19:
                        for col in range(coordinate[0], coordinate[0] - 3, -1):
                            if 0 <= col <= 14:
                                if obstacle_map[row][col] == 1:
                                    self.__imgRecMap[row][col][0] = image_id
                                    self.__imgRecMap[row][col][1] += 1
                                    self.__imgRecMap[row][col][2] = robot_bearing
                                    break
            elif robot_bearing == Bearing.EAST:
                for col in range(coordinate[0], coordinate[0] + 3):
                    if 0 <= col <= 14:
                        for row in range(coordinate[1], coordinate[1] + 3):
                            if 0 <= row <= 19:
                                if obstacle_map[row][col] == 1:
                                    self.__imgRecMap[row][col][0] = image_id
                                    self.__imgRecMap[row][col][1] += 1
                                    self.__imgRecMap[row][col][2] = robot_bearing
                                    break
            elif robot_bearing == Bearing.SOUTH:
                for row in range(coordinate[1], coordinate[1] - 3, -1):
                    if 0 <= row <= 19:
                        for col in range(coordinate[0], coordinate[0] + 3):
                            if 0 <= col <= 14:
                                if obstacle_map[row][col] == 1:
                                    self.__imgRecMap[row][col][0] = image_id
                                    self.__imgRecMap[row][col][1] += 1
                                    self.__imgRecMap[row][col][2] = robot_bearing
                                    break
            else:
                for col in range(coordinate[0], coordinate[0] - 3, -1):
                    if 0 <= col <= 19:
                        for row in range(coordinate[1], coordinate[1] - 3, -1):
                            if 0 <= row <= 14:
                                if obstacle_map[row][col] == 1:
                                    self.__imgRecMap[row][col][0] = image_id
                                    self.__imgRecMap[row][col][1] += 1
                                    self.__imgRecMap[row][col][2] = robot_bearing
                                    break
        except Exception as err:
            print(f'actlImgRecogAlgo::mark_img() Error msg: {err}')
            self.finished.emit()

    @pyqtSlot(dict, list, list, list, int)
    def determine_move(self, front_left_dict, all_corners, explored_map, obstacle_map, robot_bearing):
        print()
        self.__front_left_dict = front_left_dict
        if self.__algo_status == ImgRecogAlgoStatus.SEEK_GOAL and [15, 19] in all_corners:
            self.__algo_status = ImgRecogAlgoStatus.SEEK_HOME
        elif self.__algo_status == ImgRecogAlgoStatus.SEEK_HOME and [-1, 0] in all_corners:
            self.__algo_status = ImgRecogAlgoStatus.SEARCH_OBSTACLE
        elif self.__algo_status == ImgRecogAlgoStatus.LEFT_WALL_HUG:
            robot_center = all_corners[0][:]
            robot_center[0] = robot_center[0] + 1
            robot_center[1] = robot_center[1] - 2
            if self.__initial_pos is None:
                self.__initial_pos = robot_center
                self.__no_of_left_rotation = 0
            elif self.__initial_pos == robot_center and self.__no_of_left_rotation == 4:
                self.__initial_pos = None
                self.__no_of_left_rotation = 0
                list_of_coordinates = self.obstacle_without_left_hug(explored_map, obstacle_map)
                if list_of_coordinates:
                    self.__algo_status = ImgRecogAlgoStatus.SEARCH_OBSTACLE
                else:
                    self.__stop = True

        if not self.__stop:
            if can_calibrate(robot_bearing, obstacle_map, all_corners):
                self.signalSendMsg(pack_msg('c'))
            if self.__algo_status == ImgRecogAlgoStatus.SEARCH_OBSTACLE:
                list_of_coordinates = self.obstacle_without_left_hug(explored_map, obstacle_map)
                # if there is obstacle without left hug, fp to nearest obstacle
                if list_of_coordinates:
                    robot_center = all_corners[0]
                    robot_center[0] = robot_center[0] + 2
                    robot_center[1] = robot_center[1] - 1
                    # robot_pos in obstacle_coordinate is in x,y coordinate
                    obstacle_coordinate = find_nearest_obstacle(list_of_coordinates, robot_center)
                    dest_node = a_star_search(robot_center, obstacle_coordinate['robot_pos'],
                                              obstacle_coordinate['bearing'], robot_bearing, explored_map, obstacle_map)
                    print(f'\n{obstacle_coordinate}\n')
                    if dest_node is None:
                        print("A STAR SEARCH is unable to search the fastest path")
                        self.__stop = True
                    else:
                        self.__algo_status = ImgRecogAlgoStatus.FP_TO_OBSTACLE
                        self.__move_cmd = gen_move_cmd(dest_node)
                        self.__move_cmd_index = -1
                        self.send_a_star_move_cmd()
                else:
                    self.__stop = True
            elif self.__algo_status == ImgRecogAlgoStatus.LEFT_WALL_HUG:
                left_obstacle = self.is_left_obstacle(robot_bearing, all_corners, obstacle_map)
                if left_obstacle and not self.__pic_taken:
                    # TODO: Actual Take Pic algorithm
                    self.signalSendMsg('TP|')
                else:
                    self.__pic_taken = False
                    if not self.__robot_just_turned_left:
                        if self.__front_left_dict['L'] == 1 and \
                                self.__front_left_dict['F'] == 0:  # if left is not free and front is free
                            self.signalSendMsg(pack_msg('1'))
                        elif self.__front_left_dict['L'] == 1 and \
                                self.__front_left_dict['F'] == 1:  # if left and front is not free
                            self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                            self.signalSendMsg(pack_msg('r'))
                        elif self.__front_left_dict['L'] == 0:  # if left is free, turn left to hug the left wall
                            self.__robot_just_turned_left = True
                            self.__no_of_left_rotation = self.__no_of_left_rotation + 1
                            self.signalSendMsg(pack_msg('l'))
                    else:
                        self.__robot_just_turned_left = False
                        self.signalSendMsg(pack_msg('1'))
            elif self.__algo_status == ImgRecogAlgoStatus.FP_TO_OBSTACLE:
                self.send_a_star_move_cmd()
            else:
                # algo_status = seek_home or seek_goal
                # at every move, check if there is an obstacle on the left.
                # if yes, take pic
                left_obstacle = self.is_left_obstacle(robot_bearing, all_corners, obstacle_map)
                if left_obstacle and not self.__pic_taken:
                    # TODO: Actual Take Pic algorithm
                    self.signalSendMsg('TP|')
                else:
                    self.__pic_taken = False
                    if not self.__robot_just_turned_left:
                        if self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 0:
                            # if left is not free and front is free
                            self.signalSendMsg(pack_msg('1'))
                        elif self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 1:
                            # if left and front is not free
                            self.signalSendMsg(pack_msg('r'))
                        elif self.__front_left_dict['L'] == 0:
                            # if left is free, turn left to hug the left wall
                            self.__robot_just_turned_left = True
                            self.signalSendMsg(pack_msg('l'))
                    else:
                        self.__robot_just_turned_left = False
                        self.signalSendMsg(pack_msg('1'))
        else:
            for row_item in self.__imgRecMap:
                print(row_item)
            print()
            try:
                for img in self.__img_id:
                    possible_img_pos = []
                    for row in range(len(self.__imgRecMap)):
                        for col in range(len(self.__imgRecMap[row])):
                            if self.__imgRecMap[row][col][0] == img:
                                temp = self.__imgRecMap[row][col][:]
                                temp.append(row)
                                temp.append(col)
                                possible_img_pos.append(temp)
                    freq_occr = [possible_img_pos[0][1], 1]  # [highest freq, no of occr of said freq]
                    for i in range(1, len(possible_img_pos)):
                        if possible_img_pos[i][1] == freq_occr[0]:
                            freq_occr[1] += 1
                        elif possible_img_pos[i][1] > freq_occr[0]:
                            freq_occr[0] = possible_img_pos[i][1]
                            freq_occr[1] = 1
                    print(freq_occr)
                    print(possible_img_pos)
                    if freq_occr[1] == 1:
                        for item in possible_img_pos:
                            if item[1] == freq_occr[0]:
                                print(f'img: {possible_img_pos[0][0]}, {item[4]}, {item[3]}')
                    else:
                        if possible_img_pos[0][2] == Bearing.NORTH:
                            min_row = possible_img_pos[0][3]
                            for i in range(1, len(possible_img_pos)):
                                if possible_img_pos[i][3] < min_row and freq_occr[0] == possible_img_pos[i][1]:
                                    min_row = possible_img_pos[i][3]
                            print(f'img: {possible_img_pos[0][0]}, {possible_img_pos[0][4]}, {min_row}')
                        elif possible_img_pos[0][2] == Bearing.SOUTH:
                            max_row = possible_img_pos[0][3]
                            for i in range(1, len(possible_img_pos)):
                                if possible_img_pos[i][3] > max_row and freq_occr[0] == possible_img_pos[i][1]:
                                    max_row = possible_img_pos[i][3]
                            print(f'img: {possible_img_pos[0][0]}, {possible_img_pos[0][4]}, {max_row}')
                        elif possible_img_pos[0][2] == Bearing.EAST:
                            min_col = possible_img_pos[0][4]
                            for i in range(1, len(possible_img_pos)):
                                if possible_img_pos[i][4] < min_col and freq_occr[0] == possible_img_pos[i][1]:
                                    min_col = possible_img_pos[i][4]
                            print(f'img: {possible_img_pos[0][0]}, {min_col}, {possible_img_pos[0][3]}')
                        else:
                            max_col = possible_img_pos[0][4]
                            for i in range(1, len(possible_img_pos)):
                                if possible_img_pos[i][4] > max_col and freq_occr[0] == possible_img_pos[i][1]:
                                    max_col = possible_img_pos[i][4]
                            print(f'img: {possible_img_pos[0][0]}, {max_col}, {possible_img_pos[0][3]}')
                    print()
            except Exception as err:
                print(f'actlImgRecogAlgo::determineMove() Error msg: {err}')

            print('emitting finished')
            self.finished.emit()

    def send_a_star_move_cmd(self):
        if self.__move_cmd is not None:
            self.__move_cmd_index = self.__move_cmd_index + 1
            if self.__move_cmd_index < len(self.__move_cmd):
                if self.__move_cmd[self.__move_cmd_index] == 'RR':
                    self.signalSendMsg(pack_msg('r'))
                elif self.__move_cmd[self.__move_cmd_index] == 'RL':
                    self.signalSendMsg(pack_msg('l'))
                else:
                    self.signalSendMsg(pack_msg('1'))
            else:
                self.__move_cmd_index = -1
                self.__move_cmd = None
                self.__algo_status = ImgRecogAlgoStatus.LEFT_WALL_HUG
        else:
            self.__move_cmd_index = -1

    @pyqtSlot(str)
    def save_prediction(self, result, bearing, all_corners, obstacle_map):
        self.__pic_taken = True
        if bearing == Bearing.NORTH:
            coordinate = all_corners[2]
        elif bearing == Bearing.EAST:
            coordinate = all_corners[0]
        elif bearing == Bearing.SOUTH:
            coordinate = all_corners[1]
        else:
            coordinate = all_corners[3]
        self.mark_img(bearing, coordinate, obstacle_map, result)
        self.signalSendMsg(pack_msg('s'))

    @pyqtSlot()
    def run(self):
        self.__stop = False
        self.__algo_status = ImgRecogAlgoStatus.SEEK_GOAL
        self.__robot_just_turned_left = False
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__no_of_left_rotation = 0
        self.__img_id = set()
        self.__pic_taken = False
        self.reset_map()
        self.signalSendMsg(pack_msg('s'))

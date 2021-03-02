import time

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


class SimImgRecogAlgo(QObject):
    finished = pyqtSignal()
    signalSense = pyqtSignal()
    signalMoveRobotForward = pyqtSignal()
    signalMoveRobotBackward = pyqtSignal()
    signalRotateRobotRight = pyqtSignal()
    signalRotateRobotLeft = pyqtSignal()
    signalTakePic = pyqtSignal()

    def __init__(self):
        super(SimImgRecogAlgo, self).__init__()
        self.__stop = False
        self.__time = 0.05
        self.__algo_status = ImgRecogAlgoStatus.SEEK_GOAL
        self.__robot_just_turned_left = False
        self.__front_left_dict = dict()
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__no_of_left_rotation = 0
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

    def set_time(self, sleep_time):
        self.__time = sleep_time

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

    @pyqtSlot(dict, list, list, list, int)
    def determine_move(self, front_left_dict, all_corners, explored_map, obstacle_map, robot_bearing):
        print()
        self.__front_left_dict = front_left_dict
        print(self.__algo_status)
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
                time.sleep(self.__time)
                left_obstacle = self.is_left_obstacle(robot_bearing, all_corners, obstacle_map)
                if left_obstacle:
                    self.signalTakePic.emit()
                else:
                    if not self.__robot_just_turned_left:
                        if self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 0:  # if left is not free and front is free
                            self.signalMoveRobotForward.emit()
                            self.signalSense.emit()
                        elif self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 1:  # if left and front is not free
                            self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                            self.signalRotateRobotRight.emit()
                            self.signalSense.emit()
                        elif self.__front_left_dict['L'] == 0:  # if left is free, turn left to hug the left wall
                            self.__robot_just_turned_left = True
                            self.__no_of_left_rotation = self.__no_of_left_rotation + 1
                            self.signalRotateRobotLeft.emit()
                            self.signalSense.emit()
                    else:
                        self.__robot_just_turned_left = False
                        self.signalMoveRobotForward.emit()
                        self.signalSense.emit()
            elif self.__algo_status == ImgRecogAlgoStatus.FP_TO_OBSTACLE:
                self.send_a_star_move_cmd()
            else:
                # algo_status = seek_home or seek_goal
                time.sleep(self.__time)
                # at every move, check if there is an obstacle on the left.
                # if yes, take pic
                left_obstacle = self.is_left_obstacle(robot_bearing, all_corners, obstacle_map)
                if left_obstacle:
                    self.signalTakePic.emit()
                else:
                    if not self.__robot_just_turned_left:
                        if self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 0:  # if left is not free and front is free
                            self.signalMoveRobotForward.emit()
                            self.signalSense.emit()
                        elif self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 1:  # if left and front is not free
                            self.signalRotateRobotRight.emit()
                            self.signalSense.emit()
                        elif self.__front_left_dict['L'] == 0:  # if left is free, turn left to hug the left wall
                            self.__robot_just_turned_left = True
                            self.signalRotateRobotLeft.emit()
                            self.signalSense.emit()
                    else:
                        self.__robot_just_turned_left = False
                        self.signalMoveRobotForward.emit()
                        self.signalSense.emit()
        else:
            print('emitting finished')
            self.finished.emit()

    @pyqtSlot()
    def move_robot_after_taking_pic(self):
        time.sleep(self.__time)
        if self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 0:  # if left is not free and front is free
            self.signalMoveRobotForward.emit()
            self.signalSense.emit()
        elif self.__front_left_dict['L'] == 1 and self.__front_left_dict['F'] == 1:  # if left and front is not free
            self.__no_of_left_rotation = self.__no_of_left_rotation - 1
            self.signalRotateRobotRight.emit()
            self.signalSense.emit()
        elif self.__front_left_dict['L'] == 0:  # if left is free, turn left to hug the left wall
            self.__no_of_left_rotation = self.__no_of_left_rotation + 1
            self.__robot_just_turned_left = True
            self.signalRotateRobotLeft.emit()
            self.signalSense.emit()

    def send_a_star_move_cmd(self):
        if self.__move_cmd is not None:
            self.__move_cmd_index = self.__move_cmd_index + 1
            if self.__move_cmd_index < len(self.__move_cmd):
                time.sleep(self.__time)
                if self.__move_cmd[self.__move_cmd_index] == 'RR':
                    self.signalRotateRobotRight.emit()
                elif self.__move_cmd[self.__move_cmd_index] == 'RL':
                    self.signalRotateRobotLeft.emit()
                else:
                    self.signalMoveRobotForward.emit()
                self.signalSense.emit()
            else:
                self.__move_cmd_index = -1
                self.__move_cmd = None
                self.__algo_status = ImgRecogAlgoStatus.LEFT_WALL_HUG
                self.signalSense.emit()
        else:
            self.__move_cmd_index = -1

    @pyqtSlot()
    def run(self):
        self.__stop = False
        self.__algo_status = ImgRecogAlgoStatus.SEEK_GOAL
        self.__robot_just_turned_left = False
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__no_of_left_rotation = 0
        self.reset_map()
        self.signalSense.emit()

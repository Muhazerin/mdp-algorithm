# TODO: Need stop signal to signal the arduino and android that the expl has stopped
# TODO: Hardcode the timer to 330 seconds (5 mins 30 secs)
#   After timer timeout, FP back to home
import math
from collections import Counter

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from constants import AlgoStatus, Bearing
from simExplFastPath import a_star_search, gen_move_cmd, get_nearest_goal


def is_everything_explored(explored_map):
    for row in range(len(explored_map)):
        for col in range(len(explored_map[row])):
            if explored_map[row][col] == 0:
                return False
    return True


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


class ActlExplAlgo(QObject):
    finished = pyqtSignal()
    signalSendMsg = pyqtSignal(str)

    def __init__(self):
        super(ActlExplAlgo, self).__init__()
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.__robotJustTurnedLeft = False
        self.__stop = False
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__coverage = 100
        self.__no_of_left_rotation = 0

    @pyqtSlot()
    def timer_timeout(self):
        print('Actual Exploration 5m30s passed. FP to Home')
        self.__algoStatus = AlgoStatus.FP_HOME_SEARCH

    def send_msg(self, msg):
        cmd = 'EC|' + msg
        self.signalSendMsg.emit(cmd)

    @pyqtSlot(dict, list, list, list, int)
    def determineMove(self, frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing):
        print()
        coun = Counter()
        for row in exploredMap:
            coun.update(row)
        coverage_per = math.floor(coun[1] / 3)
        print(f'[Actual Exploration Algorithm] Coverage Percentage: {coverage_per}')
        if coverage_per >= self.__coverage and self.__algoStatus != AlgoStatus.FP_HOME_SEEK:
            self.__algoStatus = AlgoStatus.FP_HOME_SEARCH
        elif self.__algoStatus == AlgoStatus.SEEK_GOAL and [15, 19] in allCorners:
            self.__algoStatus = AlgoStatus.SEEK_HOME
        elif self.__algoStatus == AlgoStatus.SEEK_HOME and [-1, 0] in allCorners:
            if coverage_per < self.__coverage:
                self.__algoStatus = AlgoStatus.FP_UNEXPLORED_SEARCH
            else:
                self.__stop = True
        elif self.__algoStatus == AlgoStatus.LEFT_WALL_HUG:
            robot_center = allCorners[0][:]
            robot_center[0] = robot_center[0] + 1
            robot_center[1] = robot_center[1] - 2
            if self.__initial_pos is None:
                self.__initial_pos = robot_center
                self.__no_of_left_rotation = 0
            elif self.__initial_pos == robot_center and self.__no_of_left_rotation == 4:
                self.__initial_pos = None
                self.__no_of_left_rotation = 0
                if coverage_per < self.__coverage:
                    self.__algoStatus = AlgoStatus.FP_UNEXPLORED_SEARCH
                else:
                    self.__algoStatus = AlgoStatus.FP_HOME_SEARCH

        if not self.__stop:
            if self.__algoStatus == AlgoStatus.FP_HOME_SEARCH:
                goal = [2, 2]
                facing = Bearing.WEST
                print('[Actual Exploration Algorithm] A* to Home')
                dest_node = a_star_search([allCorners[0][0] + 2, allCorners[0][1] - 1], goal, facing, robotBearing,
                                          exploredMap, obstacleMap)
                if dest_node is None:
                    print("[Actual Exploration Algorithm] WTF!! A STAR SEARCH IS UNABLE TO SEARCH FAST PATH TO HOME?!")
                    self.__stop = True
                    self.finished.emit()
                else:
                    self.__algoStatus = AlgoStatus.FP_HOME_SEEK
                    self.__move_cmd = gen_move_cmd(dest_node)
                    self.__move_cmd_index = -1
                    self.send_a_star_move_cmd_no_sense()
            elif self.__algoStatus == AlgoStatus.FP_UNEXPLORED_SEARCH:
                robot_center = allCorners[0][:]
                robot_center[0] = robot_center[0] + 2
                robot_center[1] = robot_center[1] - 1
                nearest_goal = get_nearest_goal(exploredMap, obstacleMap, robot_center)
                print(f'[Actual Exploration Algorithm] A* to goal: {nearest_goal["robot_pos"]}')
                dest_node = a_star_search(robot_center, nearest_goal['robot_pos'], nearest_goal['bearing'],
                                          robotBearing, exploredMap, obstacleMap)
                if dest_node is None:
                    print("[Actual Exploration Algorithm] WTF!! THERE'S UNEXPLORED CELLS AND A STAR SEARCH RETURNS NONE?!")
                    self.__stop = True
                    self.finished.emit()
                else:
                    self.__algoStatus = AlgoStatus.FP_UNEXPLORED_SEEK
                    self.__move_cmd = gen_move_cmd(dest_node)
                    self.__move_cmd_index = -1
                    self.send_a_star_move_cmd()
            elif self.__algoStatus == AlgoStatus.FP_UNEXPLORED_SEEK:
                self.send_a_star_move_cmd()
            elif self.__algoStatus == AlgoStatus.FP_HOME_SEEK:
                self.send_a_star_move_cmd_no_sense()
            else:
                if not self.__robotJustTurnedLeft:
                    if frontLeftDict['L'] == 1 and frontLeftDict['F'] == 0:  # if left is not free and front is free
                        self.send_msg('1')
                    elif frontLeftDict['L'] == 1 and frontLeftDict['F'] == 1:  # if left and front is not free
                        self.__no_of_left_rotation -= 1
                        self.send_msg('r')
                    elif frontLeftDict['L'] == 0:  # if left is free, turn left to hug the left wall
                        self.__robotJustTurnedLeft = True
                        self.__no_of_left_rotation += 1
                        self.send_msg('l')
                else:
                    self.__robotJustTurnedLeft = False
                    self.send_msg('1')
        else:
            self.finished.emit()

    def send_a_star_move_cmd(self):
        if self.__move_cmd is not None:
            self.__move_cmd_index = self.__move_cmd_index + 1
            if self.__move_cmd_index < len(self.__move_cmd):
                if self.__move_cmd[self.__move_cmd_index] == 'RR':
                    self.send_msg('r')
                elif self.__move_cmd[self.__move_cmd_index] == 'RL':
                    self.send_msg('l')
                else:
                    self.send_msg('1')
            else:
                self.__move_cmd_index = -1
                self.__move_cmd = None
                self.__algoStatus = AlgoStatus.LEFT_WALL_HUG
                self.signalSendMsg('s')
        else:
            self.__move_cmd_index = -1

    @pyqtSlot()
    def send_a_star_move_cmd_no_sense(self):
        if self.__move_cmd is not None:
            self.__move_cmd_index = self.__move_cmd_index + 1
            if self.__move_cmd_index < len(self.__move_cmd):
                if self.__move_cmd[self.__move_cmd_index] == 'RR':
                    self.send_msg('r')
                elif self.__move_cmd[self.__move_cmd_index] == 'RL':
                    self.send_msg('l')
                else:
                    self.send_msg('1')
            else:
                self.__move_cmd_index = -1
                self.__move_cmd = None
                self.__stop = True
                self.finished.emit()
        else:
            self.__move_cmd_index = -1

    @pyqtSlot()
    def run(self):
        print('Actual Exploration Started')
        self.__stop = False
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.__initial_pos = None
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__no_of_left_rotation = 0
        self.send_msg('s')

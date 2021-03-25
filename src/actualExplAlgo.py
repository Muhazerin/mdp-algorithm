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


class ActlExplAlgo(QObject):
    finished = pyqtSignal()
    signalSendMsg = pyqtSignal(str)
    signalDetermineMove = pyqtSignal()

    def __init__(self):
        super(ActlExplAlgo, self).__init__()
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.__robot_just_turned_left = False
        self.__robot_just_turned_left_and_move_forward = False
        self.__phantom_block_loop = False
        self.__stop = False
        self.__move_cmd = None
        self.__move_cmd_index = -1
        self.__initial_pos = None
        self.__front_left_dict = None
        self.__coverage = 100
        self.__no_of_left_rotation = 0

    @pyqtSlot()
    def timer_timeout(self):
        print('Actual Exploration 5m30s passed. FP to Home')
        self.__algoStatus = AlgoStatus.FP_HOME_SEARCH
        self.signalDetermineMove.emit()

    def send_msg(self, msg):
        cmd = 'EC|' + msg
        self.signalSendMsg.emit(cmd)

    def normal_robot_movement(self):
        # normal robot movement algorithm
        print(self.__front_left_dict)
        if self.__robot_just_turned_left_and_move_forward:
            print('just turn left, move forward')
            self.__robot_just_turned_left_and_move_forward = False
            if self.__front_left_dict['L'] == 0:  # Phantom Block Loop Detected
                self.__phantom_block_loop = True
                self.__no_of_left_rotation = self.__no_of_left_rotation + 1
                self.send_msg('l')
            else:
                if self.__front_left_dict['F'] == 1:
                    if self.__front_left_dict['R'] == 0:
                        self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                        self.send_msg('r')
                    else:
                        self.__no_of_left_rotation -= 2
                        self.send_msg('u')
                else:
                    self.send_msg('1')
        elif self.__phantom_block_loop:
            print('phantom loop detected')
            if self.__front_left_dict['F'] == 0:  # if front is free in phantom_block_loop
                self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                self.send_msg('1')
            else:  # if front is not empty
                self.send_msg('r')
                self.__phantom_block_loop = False
        elif not self.__robot_just_turned_left:     # normal movement
            print('normal movement')
            if self.__front_left_dict['L'] == 1 and \
                    self.__front_left_dict['F'] == 0:  # if left is not free and front is free
                self.send_msg('1')
            elif self.__front_left_dict['L'] == 1 and \
                    self.__front_left_dict['F'] == 1:  # if left and front is not free
                if self.__front_left_dict['R'] == 0:
                    self.__no_of_left_rotation = self.__no_of_left_rotation - 1
                    self.send_msg('r')
                else:
                    self.__no_of_left_rotation -= 2
                    self.send_msg('u')  # u-turn msg
            elif self.__front_left_dict['L'] == 0:  # if left is free, turn left to hug the left wall
                self.__robot_just_turned_left = True
                self.__no_of_left_rotation = self.__no_of_left_rotation + 1
                self.send_msg('l')
        else:   # just turn left
            print('just turn left')
            self.__robot_just_turned_left = False
            self.__robot_just_turned_left_and_move_forward = True
            self.send_msg('1')

    @pyqtSlot(dict, list, list, list, int)
    def determineMove(self, frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing):
        print()
        self.__front_left_dict = frontLeftDict
        coun = Counter()
        for row in exploredMap:
            coun.update(row)
        coverage_per = math.floor(coun[1] / 3)
        print(f'[Actual Exploration Algorithm] Coverage Percentage: {coverage_per}')
        if coverage_per >= self.__coverage and self.__algoStatus != AlgoStatus.FP_HOME_SEEK:
            # if coverage is >= 100 and the algo status is not fp_home_seek, change algo status to fp_home_search
            # so the algo can return home
            self.__algoStatus = AlgoStatus.FP_HOME_SEARCH
        elif self.__algoStatus == AlgoStatus.SEEK_GOAL and [15, 19] in allCorners:
            # if algo status is seek_goal and robot is at goal, change status to seek_home
            self.__algoStatus = AlgoStatus.SEEK_HOME
        elif self.__algoStatus == AlgoStatus.SEEK_HOME and [-1, 0] in allCorners:
            # if algo status is seek_home and robot is at home, check the coverage
            if coverage_per < self.__coverage:
                # if coverage is less than desired, fp to nearest unexplored
                self.__algoStatus = AlgoStatus.FP_UNEXPLORED_SEARCH
            else:
                # else just stop
                self.__stop = True
        elif self.__algoStatus == AlgoStatus.LEFT_WALL_HUG:
            # if the algo status is left_wall_hug, need to check some things
            # like whether it has looped 1 round around the obstacle
            robot_center = allCorners[0][:]
            robot_center[0] = robot_center[0] + 1
            robot_center[1] = robot_center[1] - 2
            if self.__initial_pos is None:
                # just reached the obstacle
                self.__initial_pos = robot_center
                self.__no_of_left_rotation = 0
            elif self.__initial_pos == robot_center and self.__no_of_left_rotation == 4:
                # robot has looped around the obstacle
                self.__initial_pos = None
                self.__no_of_left_rotation = 0
                if coverage_per < self.__coverage:
                    # if coverage is less than desired, fp to nearest unexplored
                    self.__algoStatus = AlgoStatus.FP_UNEXPLORED_SEARCH
                else:
                    # else, fp to home
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
                print('determine_move::fp to nearest unexplored obstacle')
                self.send_a_star_move_cmd()
            elif self.__algoStatus == AlgoStatus.FP_HOME_SEEK:
                print('determine_move::fp to home')
                self.send_a_star_move_cmd_no_sense()
            else:
                print('determine_move::normal algo status')
                self.normal_robot_movement()
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
                elif self.__move_cmd[self.__move_cmd_index] == 'U':
                    self.send_msg('u')
                else:
                    self.send_msg('1')
            else:
                self.__move_cmd_index = -1
                self.__move_cmd = None
                self.__algoStatus = AlgoStatus.LEFT_WALL_HUG
                self.signalDetermineMove.emit()
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
                elif self.__move_cmd[self.__move_cmd_index] == 'U':
                    self.send_msg('u')
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
        self.__robot_just_turned_left = False
        self.__robot_just_turned_left_and_move_forward = False
        self.__phantom_block_loop = False
        self.__front_left_dict = None
        self.__move_cmd_index = -1
        self.__no_of_left_rotation = 0
        self.send_msg('s')

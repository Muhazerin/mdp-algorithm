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


class ActlExplAlgoLWH(QObject):
    finished = pyqtSignal()
    signalSendMsg = pyqtSignal(str)
    signalDetermineMove = pyqtSignal()

    def __init__(self):
        super(ActlExplAlgoLWH, self).__init__()
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
        self.__stop = True
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
        if self.__algoStatus == AlgoStatus.SEEK_GOAL and [15, 19] in allCorners:
            # if algo status is seek_goal and robot is at goal, change status to seek_home
            self.__algoStatus = AlgoStatus.SEEK_HOME
        elif self.__algoStatus == AlgoStatus.SEEK_HOME and [-1, 0] in allCorners:
                self.__stop = True

        if not self.__stop:
            print('determine_move::normal algo status')
            self.normal_robot_movement()
        else:
            self.finished.emit()

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

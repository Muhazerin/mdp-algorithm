import math

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import time
from collections import Counter
from constants import AlgoStatus, Bearing
from simExplFastPath import find_valid_unexplored, a_star_search, gen_move_cmd


class ActlExplAlgo(QObject):
    finished = pyqtSignal()
    signalSense = pyqtSignal()
    signalMoveRobotForward = pyqtSignal()
    signalMoveRobotBackward = pyqtSignal()
    signalRotateRobotRight = pyqtSignal()
    signalRotateRobotLeft = pyqtSignal()

    def __init__(self):
        super(ActlExplAlgo, self).__init__()
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.__robotJustTurnedLeft = False
        self.__stop = False
        self.__time = 0.05
        self.__coverage = 0
        self.__move_cmd = None
        self.__move_cmd_index = -1
        # self.signalSense.emit()

    def set_time(self, sleep_time):
        self.__time = sleep_time

    def set_coverage(self, coverage):
        self.__coverage = coverage

    @pyqtSlot()
    def timer_timeout(self):
        print('Timer timeout')
        self.__algoStatus = AlgoStatus.FP_HOME_SEARCH

    @pyqtSlot(dict, list, list, list, int)
    def determineMove(self, frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing):
        print()
        coun = Counter()
        for row in exploredMap:
            coun.update(row)
        coverage_per = math.floor(coun[1] / 3)
        print(f'Coverage Percentage: {coverage_per}')
        if coverage_per >= self.__coverage and self.__algoStatus != AlgoStatus.FP_HOME_SEEK:
            self.__algoStatus = AlgoStatus.FP_HOME_SEARCH
        elif self.__algoStatus == AlgoStatus.SEEK_GOAL and [15, 19] in allCorners:
            self.__algoStatus = AlgoStatus.SEEK_HOME
        elif self.__algoStatus == AlgoStatus.SEEK_HOME and [-1, 0] in allCorners:
            if coverage_per < self.__coverage:
                self.__algoStatus = AlgoStatus.FP_UNEXPLORED_SEARCH
            else:
                self.__stop = True
        elif self.__algoStatus == AlgoStatus.FP_UNEXPLORED_FINISHED:
            if coverage_per < self.__coverage:
                self.__algoStatus = AlgoStatus.FP_UNEXPLORED_SEARCH
            else:
                self.__algoStatus = AlgoStatus.FP_HOME_SEARCH

        if not self.__stop:
            if self.__algoStatus == AlgoStatus.FP_HOME_SEARCH:
                goal = [2, 2]
                facing = Bearing.WEST
                print('A* to Home')
                dest_node = a_star_search([allCorners[0][0] + 2, allCorners[0][1] - 1], goal, facing, robotBearing,
                                          exploredMap, obstacleMap)
                if dest_node is None:
                    print("WTF!! A STAR SEARCH IS UNABLE TO SEARCH FAST PATH TO HOME?!")
                    self.__stop = True
                    self.finished.emit()
                else:
                    self.__algoStatus = AlgoStatus.FP_HOME_SEEK
                    self.__move_cmd = gen_move_cmd(dest_node)
                    self.__move_cmd_index = -1
                    self.send_a_star_move_cmd()
            elif self.__algoStatus == AlgoStatus.FP_UNEXPLORED_SEARCH:
                goal, facing = find_valid_unexplored(exploredMap, obstacleMap)
                if goal is None or facing is None:
                    print('Error! There\'s an unreachable unexplored cell')
                    self.__stop = True
                    self.finished.emit()
                else:
                    print(f'A* to goal: {goal}')
                    dest_node = a_star_search([allCorners[0][0] + 2, allCorners[0][1] - 1], goal, facing, robotBearing,
                                              exploredMap, obstacleMap)
                    if dest_node is None:
                        print("WTF!! THERE'S UNEXPLORED CELLS AND A STAR SEARCH RETURNS NONE?!")
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
                self.send_a_star_move_cmd()
            else:
                time.sleep(self.__time)
                if not self.__robotJustTurnedLeft:
                    if frontLeftDict['L'] == 1 and frontLeftDict['F'] == 0:  # if left is not free and front is free
                        self.signalMoveRobotForward.emit()
                        self.signalSense.emit()
                    elif frontLeftDict['L'] == 1 and frontLeftDict['F'] == 1:  # if left and front is not free
                        self.signalRotateRobotRight.emit()
                        self.signalSense.emit()
                    elif frontLeftDict['L'] == 0:  # if left is free, turn left to hug the left wall
                        self.__robotJustTurnedLeft = True
                        self.signalRotateRobotLeft.emit()
                        self.signalSense.emit()
                else:
                    self.__robotJustTurnedLeft = False
                    self.signalMoveRobotForward.emit()
                    self.signalSense.emit()
        else:
            self.finished.emit()

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
                if self.__algoStatus == AlgoStatus.FP_HOME_SEEK:
                    self.__algoStatus = AlgoStatus.FP_HOME_FINISHED
                    self.__stop = True
                    self.finished.emit()
                else:
                    self.__algoStatus = AlgoStatus.FP_UNEXPLORED_FINISHED
                    self.signalSense.emit()
        else:
            self.__move_cmd_index = -1

    @pyqtSlot()
    def run(self):
        self.__stop = False
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.signalSense.emit()

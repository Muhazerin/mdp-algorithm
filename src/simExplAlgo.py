from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import time
from enum import IntEnum

# TODO: Robot needs to know that it reaches GOAL
# TODO: Robot needs to know when to stop
#  (i.e. after reaching GOAL at least once, if it reaches START, it will stop)
#  (idea: if coverage < user specified coverage, change exploration algo from left wall hugging to seek unexplored)
TIME = 0.01


class AlgoStatus(IntEnum):
    SEEK_GOAL = 0
    SEEK_HOME = 1


class SimExplAlgo(QObject):
    finished = pyqtSignal()
    signalSense = pyqtSignal()
    signalMoveRobotForward = pyqtSignal()
    signalMoveRobotBackward = pyqtSignal()
    signalRotateRobotRight = pyqtSignal()
    signalRotateRobotLeft = pyqtSignal()

    def __init__(self):
        super(SimExplAlgo, self).__init__()
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.__robotJustTurnedLeft = False
        self.__stop = False
        self.signalSense.emit()

    pyqtSlot(dict, list)
    def determineMove(self, frontLeftDict, allCorners):
        print()
        if self.__algoStatus == AlgoStatus.SEEK_GOAL and [15, 19] in allCorners:
            self.__algoStatus = AlgoStatus.SEEK_HOME
        elif self.__algoStatus == AlgoStatus.SEEK_HOME and [-1, 0] in allCorners:
            self.__stop = True
            self.finished.emit()


        if not self.__stop:
            time.sleep(TIME)
            if not self.__robotJustTurnedLeft:
                if frontLeftDict['L'] == 1 and frontLeftDict['F'] == 0: # if left is not free and front is free
                    self.signalMoveRobotForward.emit()
                    self.signalSense.emit()
                elif frontLeftDict['L'] == 1 and frontLeftDict['F'] == 1: # if left is not free and front is not free
                    self.signalRotateRobotRight.emit()
                    self.signalSense.emit()
                elif frontLeftDict['L'] == 0:   # if left is free, turn left to hug the left wall
                    self.__robotJustTurnedLeft = True
                    self.signalRotateRobotLeft.emit()
                    self.signalSense.emit()
            else:
                self.__robotJustTurnedLeft = False
                self.signalMoveRobotForward.emit()
                self.signalSense.emit()

    def run(self):
        self.__stop = False
        self.__algoStatus = AlgoStatus.SEEK_GOAL
        self.signalSense.emit()


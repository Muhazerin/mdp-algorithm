from PyQt5.QtCore import QObject, pyqtSignal
import time
from enum import IntEnum

# TODO: Robot needs to know that it reaches GOAL
# TODO: Robot needs to know when to stop
#  (i.e. after reaching GOAL at least once, if it reaches START, it will stop)
#  (idea: if coverage < user specified coverage, change exploration algo from left wall hugging to seek unexplored)
TIME = 0.5


class SimExplAlgo(QObject):
    finished = pyqtSignal()
    signalSense = pyqtSignal()
    signalMoveRobotForward = pyqtSignal()
    signalMoveRobotBackward = pyqtSignal()
    signalRotateRobotRight = pyqtSignal()
    signalRotateRobotLeft = pyqtSignal()

    def __init__(self, robot):
        super(SimExplAlgo, self).__init__()
        self.__robotJustTurnedLeft = False
        self.__robot = robot

    def run(self):
        for i in range(0, 90):
            self.sense()
            noOfLeftMove = self.__robot.noOfLeftMove()
            noOfForwardMove = self.__robot.noOfForwardMove()
            if not self.__robotJustTurnedLeft:
                if noOfLeftMove > 0:
                    self.rotateLeft()
                elif noOfForwardMove > 0:
                    for move in range(0, noOfForwardMove):
                        self.moveForward()
                else:
                    self.rotateRight()
            else:
                self.__robotJustTurnedLeft = False
                for move in range(0, noOfForwardMove):
                    self.moveForward()

        self.finished.emit()

    def sense(self):
        self.signalSense.emit()
        time.sleep(TIME)

    def moveForward(self):
        self.signalMoveRobotForward.emit()
        time.sleep(TIME)

    def moveBackward(self):
        self.signalMoveRobotBackward.emit()
        time.sleep(TIME)

    def rotateRight(self):
        self.signalRotateRobotRight.emit()
        time.sleep(TIME)

    def rotateLeft(self):
        self.__robotJustTurnedLeft = True
        self.signalRotateRobotLeft.emit()
        time.sleep(TIME)
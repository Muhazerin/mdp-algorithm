from PyQt5.QtCore import QObject, pyqtSignal


class RobotObject(QObject):
    signalFrontLeft = pyqtSignal(dict, list, list, list, int)
    def __init__(self):
        super(RobotObject, self).__init__()

    def emitFrontLeft(self, frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing):
        self.signalFrontLeft.emit(frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing)

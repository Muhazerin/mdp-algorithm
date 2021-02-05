from PyQt5.QtCore import QObject, pyqtSignal


class RobotObject(QObject):
    signalFrontLeft = pyqtSignal(dict, list)
    def __init__(self):
        super(RobotObject, self).__init__()

    def emitFrontLeft(self, frontLeftDict, allCorners):
        self.signalFrontLeft.emit(frontLeftDict, allCorners)
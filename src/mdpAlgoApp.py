# TODO:
# Left wall hug
# 1. Create the simulator
# GUI - pyqt (to be implemented)
# Algo to be implemented:
#   Fastest Path (A* Algorithm)
#   Explore (left wall)
# Code the actual thing
# Everything above
#   Image Recognition (on hold)
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QMainWindow, QGraphicsScene

from graphicsMgr import GraphicsMgr
from ui import mainwindow


class MDPAlgoApp(QMainWindow, mainwindow.Ui_MainWindow):
    def __init__(self):
        super(MDPAlgoApp, self).__init__()
        self.setupUi(self)

        # Disable app from maximising
        self.setMaximumSize(self.width(), self.height())

        # Create a QGraphicScene and attach it to gvMap
        self.__scene = QGraphicsScene()
        self.gvMap.setScene(self.__scene)

        # Let the graphicMgr handle designing the scene
        self.__graphicsMgr = GraphicsMgr(self.__scene)

        # Connect the signal and slots for the buttons
        self.btnForward.clicked.connect(self.btnForwardClicked)
        self.btnBackward.clicked.connect(self.btnBackwardClicked)
        self.btnRotateRight.clicked.connect(self.btnRotateRightClicked)
        self.btnRotateLeft.clicked.connect(self.btnRotateLeftClicked)

    @pyqtSlot()
    def btnForwardClicked(self):
        self.__graphicsMgr.moveRobotForward()

    @pyqtSlot()
    def btnBackwardClicked(self):
        self.__graphicsMgr.moveRobotBackward()

    @pyqtSlot()
    def btnRotateRightClicked(self):
        self.__graphicsMgr.rotateRobotRight()

    @pyqtSlot()
    def btnRotateLeftClicked(self):
        self.__graphicsMgr.rotateRobotLeft()

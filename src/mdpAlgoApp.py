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
from robot import SimRobot
from map import Map
# from simExplorationAlgo import SimExplorationAlgo
from ui import mainwindow
from mapDialog import MapDialog


class MDPAlgoApp(QMainWindow, mainwindow.Ui_MainWindow):
    def __init__(self):
        super(MDPAlgoApp, self).__init__()
        self.setupUi(self)

        # Disable app from maximising
        self.setMaximumSize(self.width(), self.height())

        # Create a QGraphicScene and attach it to gvMap (QGraphicsView)
        # This tells qt how to draw the square tiles and robot
        self.__scene = QGraphicsScene()
        self.gvMap.setScene(self.__scene)

        # Initialize the map
        self.__map = Map()
        self.__mapDialog = MapDialog(self.__map)

        # Initialize the robot
        self.__robot = SimRobot(0, -120, self.__map)
        # Let the graphicMgr handle designing the scene and robot
        self.__graphicsMgr = GraphicsMgr(self.__scene, self.__robot, self.__map)

        # self.__simExplAlgo = SimExplorationAlgo(self.__robot)

        # Connect the signal and slots for the buttons
        # These 4 signal and slot is for testing robot purposes
        self.btnForward.clicked.connect(self.btnForwardClicked)
        self.btnBackward.clicked.connect(self.btnBackwardClicked)
        self.btnRotateRight.clicked.connect(self.btnRotateRightClicked)
        self.btnRotateLeft.clicked.connect(self.btnRotateLeftClicked)

        self.btnSimExpl.clicked.connect(self.btnSimExplClicked)
        self.btnLoadMap.clicked.connect(self.btnLoadMapClicked)

    @pyqtSlot()
    def btnForwardClicked(self):
        self.__robot.moveRobotForward()
        self.__robot.sense()

    @pyqtSlot()
    def btnBackwardClicked(self):
        self.__robot.moveRobotBackward()
        self.__robot.sense()

    @pyqtSlot()
    def btnRotateRightClicked(self):
        self.__robot.rotateRobotRight()
        self.__robot.sense()

    @pyqtSlot()
    def btnRotateLeftClicked(self):
        self.__robot.rotateRobotLeft()
        self.__robot.sense()

    @pyqtSlot()
    def btnSimExplClicked(self):
        self.__simExplAlgo.start()

    @pyqtSlot()
    def btnLoadMapClicked(self):
        self.__mapDialog.exec()
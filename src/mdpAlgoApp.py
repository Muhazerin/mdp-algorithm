# TODO:
#  Coverage figure exploration
#  SPS and time-limited exploration
#  Algo to be implemented:
#    Fastest Path (A* Algorithm)
#    Explore (left wall)
#  Code the actual thing
#  Everything above
#    Image Recognition (on hold)
from PyQt5.QtCore import pyqtSlot, QThread
from PyQt5.QtWidgets import QMainWindow, QGraphicsScene, QMessageBox

from graphicsMgr import GraphicsMgr
from map import Map
from simExplAlgo import SimExplAlgo
from ui import mainwindow
from mapDialog import MapDialog


class MDPAlgoApp(QMainWindow, mainwindow.Ui_MainWindow):
    def __init__(self):
        super(MDPAlgoApp, self).__init__()
        self.setupUi(self)
        # Start and Goal coordinate
        self.__dontTouchMapList = [
            [0, 0], [1, 0], [2, 0],
            [0, 1], [1, 1], [2, 1],
            [0, 2], [1, 2], [2, 2],
            [12, 17], [13, 17], [14, 17],
            [12, 18], [13, 18], [14, 18],
            [12, 19], [13, 19], [14, 19],
        ]

        # Disable app from maximising
        self.setMaximumSize(self.width(), self.height())

        # Create a QGraphicScene and attach it to gvMap (QGraphicsView)
        # This tells qt how to draw the square tiles and robot
        self.__scene = QGraphicsScene()
        self.gvMap.setScene(self.__scene)

        # Initialize the map
        self.__map = Map()
        # Initialise the mapDialog. This allows the user to load map from disk
        self.__mapDialog = MapDialog(self.__map)

        # Let the graphicMgr handle designing the scene and robot
        self.__graphicsMgr = GraphicsMgr(self.__scene, self.__map)

        # MapDialog settings signal and slot
        self.btnLoadMap.clicked.connect(self.btnLoadMapClicked)
        self.btnResetMap.clicked.connect(self.btnResetMapClicked)
        self.btnSetWaypoint.clicked.connect(self.btnSetWaypointClicked)
        self.__mapDialog.enableWaypointSignal.connect(self.enableWaypoint)

        # simExplAlgo
        self.__thread = QThread()
        self.__simExplAlgo = SimExplAlgo()
        self.__simExplAlgo.moveToThread(self.__thread)
        self.__thread.started.connect(self.__simExplAlgo.run)
        self.__simExplAlgo.finished.connect(self.__thread.quit)
        self.__simExplAlgo.signalSense.connect(self.__graphicsMgr.simRobotSense)
        self.__simExplAlgo.signalMoveRobotForward.connect(self.__graphicsMgr.moveSimRobotForward)
        self.__simExplAlgo.signalMoveRobotBackward.connect(self.__graphicsMgr.moveSimRobotBackward)
        self.__simExplAlgo.signalRotateRobotRight.connect(self.__graphicsMgr.rotateSimRobotRight)
        self.__simExplAlgo.signalRotateRobotLeft.connect(self.__graphicsMgr.rotateSimRobotLeft)
        self.__graphicsMgr.signalFrontLeft.connect(self.__simExplAlgo.determineMove)
        self.__simExplAlgo.finished.connect(self.__thread.quit)

        self.btnSimExpl.clicked.connect(self.btnSimExplClicked)

    @pyqtSlot()
    def btnSimExplClicked(self):
        self.__thread.start()

    @pyqtSlot()
    def btnLoadMapClicked(self):
        self.__mapDialog.exec()


    @pyqtSlot()
    def btnResetMapClicked(self):
        self.__map.resetMap()
        self.leXWaypoint.setText("")
        self.leYWaypoint.setText("")

        self.leXWaypoint.setEnabled(False)
        self.leYWaypoint.setEnabled(False)
        self.btnSetWaypoint.setEnabled(False)

        self.btnSimExpl.setEnabled(True)
        self.btnSimFastPath.setEnabled(False)
        self.__graphicsMgr.resetRobot()

    @pyqtSlot()
    def enableWaypoint(self):
        self.leXWaypoint.setEnabled(True)
        self.leYWaypoint.setEnabled(True)
        self.btnSetWaypoint.setEnabled(True)
        self.btnSimExpl.setEnabled(False)

    def waypointError(self, errorMsg):
        self.__map.clearWaypoint()
        self.btnSimFastPath.setEnabled(False)
        QMessageBox.critical(self, self.windowTitle(), errorMsg)

    @pyqtSlot()
    def btnSetWaypointClicked(self):
        try:
            if self.leXWaypoint.text() == "" or self.leYWaypoint.text() == "":
                QMessageBox.critical(self, self.windowTitle(), "Invalid Waypoint")
            else:
                coordinate = [int(self.leXWaypoint.text()) - 1, int(self.leYWaypoint.text()) - 1]

                if coordinate[0] < 0 or coordinate[1] < 0 or coordinate[0] > 14 or coordinate[1] > 14:
                    self.waypointError("Waypoint out of bound")
                elif coordinate in self.__dontTouchMapList:
                    self.waypointError("Unable to set waypoint on START/GOAL")
                elif self.__map.obstacleMap[coordinate[1]][coordinate[0]] == 1:
                    self.waypointError("Unable to set waypoint on obstacle")
                else:
                    self.__map.clearWaypoint()
                    self.__map.waypoint = coordinate
                    self.btnSimFastPath.setEnabled(True)
        except Exception as err:
            print(f"[Error] mdpAlgoApp::btnSetWaypointClicked! Error msg: {err}")

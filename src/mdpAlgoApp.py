# TODO:
#  Coverage figure exploration
#  SPS and time-limited exploration
#  Algo to be implemented:
#    Fastest Path (A* Algorithm)
#    Explore (left wall)
#  Code the actual thing
#  Everything above
#    Image Recognition (on hold)
from PyQt5.QtCore import pyqtSlot, QThread, QTimer
from PyQt5.QtWidgets import QMainWindow, QGraphicsScene, QMessageBox

from graphicsMgr import GraphicsMgr
from map import Map
from simExplAlgo import SimExplAlgo
from simFastPathAlgo import SimFastPathAlgo
from ui import mainwindow
from mapDialog import MapDialog
from tcpClient import TcpClient


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

        self.__binToHexConverterDict = {
            '0000': '0',
            '0001': '1',
            '0010': '2',
            '0011': '3',
            '0100': '4',
            '0101': '5',
            '0110': '6',
            '0111': '7',
            '1000': '8',
            '1001': '9',
            '1010': 'A',
            '1011': 'B',
            '1100': 'C',
            '1101': 'D',
            '1110': 'E',
            '1111': 'F'
        }

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
        self.__simExplAlgo = None

        # Initialise the timer for time-limited exploration
        self.__qTimer = QTimer()
        self.__qTimer.setSingleShot(True)

        # simFastPathAlgo
        self.__pathThread = QThread()
        self.__simFastPathAlgo = SimFastPathAlgo()
        self.__simFastPathAlgo.moveToThread(self.__pathThread)
        self.__pathThread.started.connect(self.__simFastPathAlgo.run)
        self.__simFastPathAlgo.finished.connect(self.__pathThread.quit)
        self.__simFastPathAlgo.signalSense.connect(self.__graphicsMgr.simRobotSense)
        self.__simFastPathAlgo.signalMoveRobotForward.connect(self.__graphicsMgr.moveSimRobotForward)
        self.__simFastPathAlgo.signalMoveRobotBackward.connect(self.__graphicsMgr.moveSimRobotBackward)
        self.__simFastPathAlgo.signalRotateRobotRight.connect(self.__graphicsMgr.rotateSimRobotRight)
        self.__simFastPathAlgo.signalRotateRobotLeft.connect(self.__graphicsMgr.rotateSimRobotLeft)
        # self.__graphicsMgr.signalFrontLeft.connect(self.__simFastPathAlgo.determineMove)
        self.__simFastPathAlgo.finished.connect(self.__pathThread.quit)

        self.btnSimExpl.clicked.connect(self.btnSimExplClicked)
        self.btnSimFastPath.clicked.connect(self.btnSimFastPathClicked)

        # Tcp Client Initialisation
        self.__tcpThread = QThread()
        self.__tcpClient = TcpClient()
        self.__tcpClient.moveToThread(self.__tcpThread)
        self.__tcpThread.started.connect(self.__tcpClient.start_client)
        self.__tcpClient.finished.connect(self.__tcpThread.quit)
        self.btnRobotConnection.clicked.connect(self.btnRobotConnectionClicked)
        self.__tcpClient.finished.connect(lambda: self.btnRobotConnection.setText('Connect'))

    @pyqtSlot()
    def btnRobotConnectionClicked(self):
        if self.btnRobotConnection.text() == 'Connect':
            self.__tcpThread.start()
            self.btnRobotConnection.setText('Disconnect')
        else:
            self.__tcpClient.stop_client()

    # The creation of simExplAlgo is shifted here to eliminate threading errors
    @pyqtSlot()
    def btnSimExplClicked(self):
        self.__simExplAlgo = SimExplAlgo()
        self.__simExplAlgo.moveToThread(self.__thread)
        # Signal-Slot for thread management
        self.__thread.started.connect(self.__simExplAlgo.run)
        self.__simExplAlgo.finished.connect(lambda: print('SimExplAlgo Stopping'))
        self.__simExplAlgo.finished.connect(self.__thread.quit)
        self.__simExplAlgo.finished.connect(self.generateMapDescriptor)
        self.__thread.finished.connect(self.__simExplAlgo.deleteLater)
        # Signal-Slot for Exploration Robot Movement
        self.__simExplAlgo.signalSense.connect(self.__graphicsMgr.simRobotSense)
        self.__simExplAlgo.signalMoveRobotForward.connect(self.__graphicsMgr.moveSimRobotForward)
        self.__simExplAlgo.signalMoveRobotBackward.connect(self.__graphicsMgr.moveSimRobotBackward)
        self.__simExplAlgo.signalRotateRobotRight.connect(self.__graphicsMgr.rotateSimRobotRight)
        self.__simExplAlgo.signalRotateRobotLeft.connect(self.__graphicsMgr.rotateSimRobotLeft)
        self.__graphicsMgr.signalFrontLeft.connect(self.__simExplAlgo.determineMove)
        # Signal-Slot for FastPath back to Home
        self.__simExplAlgo.signalAstarCmd.connect(self.__graphicsMgr.interpretAstarCmd)
        self.__graphicsMgr.signalNextAstarCmd.connect(self.__simExplAlgo.send_a_star_move_cmd_no_sense)
        # Signal-Slot for timer timeout
        self.__qTimer.timeout.connect(self.__simExplAlgo.timer_timeout)

        if int(self.leSPS.text()) < 0:
            self.__simExplAlgo.set_time(0.05)
        else:
            self.__simExplAlgo.set_time(1 / int(self.leSPS.text()))
        if 0 <= int(self.leCoverageFigure.text()) <= 100:
            self.__simExplAlgo.set_coverage(int(self.leCoverageFigure.text()))
        else:
            self.__simExplAlgo.set_coverage(100)
        if int(self.leTimeLimit.text()) < 0:
            self.__qTimer.setInterval(360 * 1000)
        else:
            self.__qTimer.setInterval(int(self.leTimeLimit.text()) * 1000)

        print('Sim Exploration Start')
        self.__qTimer.start()
        self.__thread.start()

    @pyqtSlot()
    def btnSimFastPathClicked(self):
        self.__pathThread.start()


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
                coordinate = (int(self.leXWaypoint.text()) - 1, int(self.leYWaypoint.text()) - 1)

                if coordinate[0] < 0 or coordinate[1] < 0 or coordinate[0] > 14 or coordinate[1] > 19:
                    self.waypointError("Waypoint out of bound")
                elif coordinate in self.__dontTouchMapList:
                    self.waypointError("Unable to set waypoint on START/GOAL")
                elif self.__map.obstacleMap[coordinate[1]][coordinate[0]] == 1:
                    self.waypointError("Unable to set waypoint on obstacle")
                else:
                    self.__map.clearWaypoint()
                    self.__map.waypoint = coordinate
                    self.__shortestPath = self.__simFastPathAlgo.gen_full_path(self.__map.obstacleMap, self.__map.waypoint)
                    print(self.__shortestPath)
                    self.btnSimFastPath.setEnabled(True)
        except Exception as err:
            print(f"[Error] mdpAlgoApp::btnSetWaypointClicked! Error msg: {err}")

    def mapToHex(self, pStr):
        pHex = ''
        tempPstr = ''
        for index in range(0, len(pStr)):
            tempPstr += pStr[index]
            if index % 4 == 3:
                pHex += self.__binToHexConverterDict[tempPstr]
                tempPstr = ''
        return pHex


    @pyqtSlot()
    def generateMapDescriptor(self):
        p1 = '11'
        p2 = ''
        for row in range(0, len(self.__map.exploredMap)):
            for col in range(0, len(self.__map.exploredMap[row])):
                if self.__map.exploredMap[row][col] == 0:
                    p1 += '0'
                else:
                    # if the map is explored, add the cell property into p2
                    p1 += '1'
                    p2 += str(self.__map.obstacleMap[row][col])
        p1 += '11'
        extra = len(p2) % 8
        padding = 0
        if extra != 0:
            padding = 8 - extra
        for i in range(0, padding):
            p2 += '0'
        p1 = self.mapToHex(p1)
        p2 = self.mapToHex(p2)
        print('\nMap Descriptor')
        print(f'p1: {p1}')
        print(f'p2: {p2}')

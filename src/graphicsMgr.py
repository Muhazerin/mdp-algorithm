from PyQt5.QtCore import QObject, pyqtSlot, pyqtSignal

from squareTile import SquareTile
from constants import TileType, MapConstant
from robotObject import RobotObject


# Handles the graphics/drawing that user see on the app
from robot import SimRobot


class GraphicsMgr(QObject):
    signalFrontLeft = pyqtSignal(dict, list, list, list, int)
    signalNextAstarCmd = pyqtSignal()

    signalStartExpl = pyqtSignal()
    signalStartFP = pyqtSignal()
    signalStartImgRecog = pyqtSignal()
    signalStopFP = pyqtSignal()
    # signalNextMove = pyqtSignal()

    def __init__(self, scene, map):
        super(GraphicsMgr, self).__init__()
        self.__shortestPath = None
        self.__spIndex = 0

        self.__scene = scene
        self.__map = map
        self.__robotObject = RobotObject()
        self.__robot = SimRobot(0, -120, self.__map, self.__robotObject)
        self.__dontTouchMapList = MapConstant.getMapStartList()
        self.__map.addObserver(self)

        self.__robotObject.signalFrontLeft.connect(self.emitFrontLeftSignal)

        self.initMap()

        # Add the robot to the scene
        self.__scene.addItem(self.__robot)
        # self.__robot.sense()

    def getRobot(self):
        return self.__robot

    # Initialize the square tiles on the map and the robot
    def initMap(self):
        # set the 15x20 map
        # Qt is weird...
        # left to right is -x to +x
        # down to up is +y to -y
        for y in range(-40, -801, -40):
            for x in range(0, 561, 40):
                row = int(abs(y) / 40) - 1
                col = int(x / 40)
                tt = None
                if self.__map.simExplObstacleMap[row][col] == 0:
                    tt = TileType.UNEXPLORED
                elif self.__map.simExplObstacleMap[row][col] == 1:
                    tt = TileType.UNEXPLORED_OBSTACLE
                square_tile = SquareTile(x, y, tt)
                self.__scene.addItem(square_tile)

        # Retrieve the tiles from the scene in descending order (0 = descending order)
        self.__tileList = self.__scene.items(0)
        # Color the START blue
        self.changeTile(MapConstant.getMapStartList(), TileType.START)
        # Color the GOAl green
        self.changeTile(MapConstant.getMapGoalList(), TileType.UNEXPLORED_GOAL)

    def changeTile(self, mapList, tt):
        for i in mapList:
            self.__tileList[i].tileType = tt

    def updateMap(self, index, tt):
        if index not in self.__dontTouchMapList:
            self.__tileList[index].tileType = tt

    def resetRobot(self):
        self.__robot.resetPos()

    @pyqtSlot()
    def moveSimRobotForward(self):
        print('SimRobot Move Forward')
        self.__robot.moveRobotForward()

    @pyqtSlot()
    def moveSimRobotBackward(self):
        print('SimRobot Move Backward')
        self.__robot.moveRobotBackward()

    @pyqtSlot()
    def rotateSimRobotRight(self):
        print('SimRobot Rotate Right')
        self.__robot.rotateRobotRight()

    @pyqtSlot()
    def rotateSimRobotLeft(self):
        print('SimRobot Rotate Left')
        self.__robot.rotateRobotLeft()

    @pyqtSlot()
    def simRobotSense(self):
        print('SimRobot Sensing')
        self.__robot.sense()

    @pyqtSlot(dict, list, list, list, int)
    def emitFrontLeftSignal(self, frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing):
        self.signalFrontLeft.emit(frontLeftDict, allCorners, exploredMap, obstacleMap, robotBearing)

    # For Sim Expl Algo return home
    @pyqtSlot(str)
    def interpretAstarCmd(self, cmd):
        if cmd == 'RR':
            print('SimRobot Rotate Right')
            self.__robot.rotateRobotRight()
        elif cmd == 'RL':
            print('SimRobot Rotate Left')
            self.__robot.rotateRobotLeft()
        else:
            print('SimRobot Move Forward')
            self.__robot.moveRobotForward()
        self.signalNextAstarCmd.emit()

    @pyqtSlot(str)
    def interpretCmd(self, msg):
        # need communication protocol for
        #   p               start fast path
        #   i               start img recog
        #   e               start expl
        #   f               forward
        #   l               rotate left
        #   r               rotate right
        #   x,y             FPW(len == 2)
        #   1,1,1,1,1,1     sensor data (len == 6)
        print(f'Msg: {msg}')
        try:
            if msg == 'p':
                print('[Android-Algo] Start FP\n')
                self.signalStartFP.emit()
            elif msg == 'i':
                print('[Android-Algo] Start Img Recog\n')
                self.signalStartImgRecog.emit()
            elif msg == 't':
                print('[Android-Algo] Start Expl\n')
                self.signalStartExpl.emit()
            elif msg == 'f':
                print('[Arduino-Algo] Move robot forward\n')
                self.__robot.moveRobotForward()
                if self.__shortestPath is not None:
                    self.__spIndex = self.__spIndex + 1
                    if self.__spIndex == len(self.__shortestPath):
                        self.__shortestPath = 0
                        self.__spIndex = 0
                        self.signalStopFP.emit()
            elif msg == 'l':
                print('[Arduino-Algo] Rotate robot left\n')
                self.__robot.rotateRobotLeft()
                if self.__shortestPath is not None:
                    self.__spIndex = self.__spIndex + 1
                    if self.__spIndex == len(self.__shortestPath):
                        self.__shortestPath = 0
                        self.__spIndex = 0
                        self.signalStopFP.emit()
            elif msg == 'r':
                print('[Arduino-Algo] Rotate robot right\n')
                self.__robot.rotateRobotRight()
                if self.__shortestPath is not None:
                    self.__spIndex = self.__spIndex + 1
                    if self.__spIndex == len(self.__shortestPath):
                        self.__shortestPath = 0
                        self.__spIndex = 0
                        self.signalStopFP.emit()
            else:
                data = msg.split(',')
                if len(data) == 2:      # FPW
                    print(f'[Android-Algo] FPW: {data}\n')
                    coordinate = [int(data[0]), int(data[1])]
                    self.__map.waypoint = coordinate
                elif len(data) == 6:    # sensor data
                    print(f'[Arduino-Algo] Sensor data: {data}\n')
                    pass
                else:
                    print('invalid msg\n')
        except Exception as err:
            print(f'graphicsMgr::interpretCmd Error msg: {err}\n')

    def setShortestPath(self, shortestPath):
        self.__shortestPath = shortestPath
        self.__spIndex = 0

    @pyqtSlot()
    def resetShortestPath(self):
        self.__shortestPath = None
        self.__spIndex = 0

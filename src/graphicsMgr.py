import ssl

import torch
from PyQt5.QtCore import QObject, pyqtSlot, pyqtSignal, QThread

from squareTile import SquareTile
from constants import TileType, MapConstant, Bearing
from robotObject import RobotObject
import detect
import time

# Handles the graphics/drawing that user see on the app
from robot import SimRobot

import time

MAX_RIGHT = 7
RIGHT_WEIGHTAGE_REDUCTION = 14
MAX_FRONT = 2
FRONT_WEIGHTAGE_REDUCTION = 50
MAX_LEFT = 3
LEFT_WEIGHTAGE_REDUCTION = 33
SLEEP_TIME = 0.5


class GraphicsMgr(QObject):
    signalFrontLeft = pyqtSignal(dict, list, list, list, int)
    signalNextAstarCmd = pyqtSignal()

    signalStartExpl = pyqtSignal()
    signalStartFP = pyqtSignal()
    signalStartImgRecog = pyqtSignal()
    signalStopFP = pyqtSignal()
    signalDetectionResult = pyqtSignal(list, int, list)
    signalNextMove = pyqtSignal(dict, list, list, list, int)
    signalSendMsg = pyqtSignal(str)
    signalAfterPhotoData = pyqtSignal(dict, list, list, list, int)
    signalSendMdf = pyqtSignal()

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

        self.__forward = Forward()
        self.__fThread = QThread()
        self.__forward.moveToThread(self.__fThread)
        self.__fThread.started.connect(self.__forward.run)
        self.__forward.finished.connect(self.__fThread.quit)
        self.__forward.signalForward.connect(lambda: self.__robot.moveRobotForward())

        # uncomment this code for img rec
        # ssl._create_default_https_context = ssl._create_unverified_context
        # self.__model = torch.hub.load('ultralytics/yolov5', 'custom', path_or_model='best.pt')

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
    def after_photo(self):
        print('after photo')
        self.signalAfterPhotoData.emit(self.__robot.get_front_left_dict(self.__robot.get_all_corners()),
                                       self.__robot.get_all_corners(), self.__map.exploredMap,
                                       self.__map.obstacleMap, self.__robot.bearing)

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
        #   t               start expl
        #   num N           move forward N times
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
            elif msg == 'u':
                print('[Arduino-Algo] U-turn')
                self.__robot.rotateRobotLeft()
                self.__robot.rotateRobotLeft()
                if self.__shortestPath is not None:
                    self.__spIndex = self.__spIndex + 1
                    if self.__spIndex == len(self.__shortestPath):
                        self.__shortestPath = 0
                        self.__spIndex = 0
                        self.signalStopFP.emit()
            elif msg == 'd':
                print('[RPi - Algo] Photo Taken. Algo detecting image...\n')
                result = detect.get_prediction(self.__model)
                print(f"[Algo] Image detected. Result: {result}")
                self.signalDetectionResult.emit(result, self.__robot.bearing, self.__robot.get_all_corners())
            elif msg == 'z':
                print('[RPi - Algo] MDF String sent... Signalling Next Move')
                # i should receive after sending mdf string
                # do it this way s sending mdf is long. might get error if 2 threads is trying to send at the same time
                self.signalNextMove.emit(self.__robot.get_front_left_dict(self.__robot.get_all_corners()),
                                         self.__robot.get_all_corners(), self.__map.exploredMap,
                                         self.__map.obstacleMap, self.__robot.bearing)
            else:
                data = msg.split(',')
                if len(data) == 2:      # FP Waypoint
                    print(f'[Android-Algo] FPW: {data}\n')
                    coordinate = [int(data[0]), int(data[1])]
                    self.__map.waypoint = coordinate
                elif len(data) == 6:    # sensor data
                    print(f'[Arduino-Algo] Sensor data: {data}\n')
                    print('update_expl_map')
                    self.update_expl_map(data, self.__robot.get_all_corners(), self.__robot.bearing)
                    print('send mdf string')
                    self.signalSendMdf.emit()
                    print('signal next move')
                    all_corners = self.__robot.get_all_corners()
                    self.signalNextMove.emit(self.__robot.get_front_left_dict(all_corners),
                                             all_corners, self.__map.exploredMap,
                                             self.__map.obstacleMap, self.__robot.bearing)
                elif len(data) == 1:
                    self.__forward.set_n(int(data[0]))
                    self.__fThread.start()

                    if self.__shortestPath is not None:
                        self.__spIndex += 1
                        if self.__spIndex == len(self.__shortestPath):
                            self.__shortestPath = 0
                            self.__spIndex = 0
                            self.signalStopFP.emit()
                else:
                    print('invalid msg\n')
        except Exception as err:
            print(f'graphicsMgr::interpretCmd Error msg: {err}\n')

    def update_right_map(self, data, coordinate, robot_bearing):
        weightage = 100
        if robot_bearing == Bearing.NORTH:
            if data == -1:
                for i in range(MAX_RIGHT):
                    if 0 <= coordinate[0] + i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] + i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] + i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] + i, weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] + i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] + i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] + i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] + i, weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] + data - 1 <= 14 and 0 <= coordinate[1] <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + data - 1]:
                        self.__map.updateExploredMap(coordinate[1], coordinate[0] + data - 1, 1)
                        self.__map.updateObstacleMap(coordinate[1], coordinate[0] + data - 1, 1)
                        self.__map.updateWeightageMap(coordinate[1], coordinate[0] + data - 1, weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0] + data - 1},{coordinate[1]}')
                        # time.sleep(SLEEP_TIME)
        elif robot_bearing == Bearing.EAST:
            if data == -1:
                for i in range(MAX_RIGHT):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] - i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] - i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] - i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] - i, coordinate[0], weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] - i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] - i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] - i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] - i, coordinate[0], weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - (data - 1) <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1] - (data - 1)][coordinate[0]]:
                        self.__map.updateExploredMap(coordinate[1] - (data - 1), coordinate[0], 1)
                        self.__map.updateObstacleMap(coordinate[1] - (data - 1), coordinate[0], 1)
                        self.__map.updateWeightageMap(coordinate[1] - (data - 1), coordinate[0], weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0]},{coordinate[1] - (data - 1)}')
                        # time.sleep(SLEEP_TIME)
        elif robot_bearing == Bearing.SOUTH:
            if data == -1:
                for i in range(MAX_RIGHT):
                    if 0 <= coordinate[0] - i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] - i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] - i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] - i, weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] - i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] - i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] - i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] - i, weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] - (data - 1) <= 14 and 0 <= coordinate[1] <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - (data - 1)]:
                        self.__map.updateExploredMap(coordinate[1], coordinate[0] - (data - 1), 1)
                        self.__map.updateObstacleMap(coordinate[1], coordinate[0] - (data - 1), 1)
                        self.__map.updateWeightageMap(coordinate[1], coordinate[0] - (data - 1), weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0] - (data - 1)},{coordinate[1]}')
                        # time.sleep(SLEEP_TIME)
        else:
            if data == -1:
                for i in range(MAX_RIGHT):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] + i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] + i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] + i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] + i, coordinate[0], weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] + i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] + i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] + i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] + i, coordinate[0], weightage)
                    weightage -= RIGHT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + data - 1 <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1] + data - 1][coordinate[0]]:
                        self.__map.updateExploredMap(coordinate[1] + data - 1, coordinate[0], 1)
                        self.__map.updateObstacleMap(coordinate[1] + data - 1, coordinate[0], 1)
                        self.__map.updateWeightageMap(coordinate[1] + data - 1, coordinate[0], weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0]},{coordinate[1] + data - 1}')
                        # time.sleep(SLEEP_TIME)

    def update_front_map(self, data, coordinate, robot_bearing):
        weightage = 100
        if robot_bearing == Bearing.NORTH:
            if data == -1:
                for i in range(MAX_FRONT):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] + i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] + i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] + i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] + i, coordinate[0], weightage)
                    weightage -= FRONT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if weightage >= self.__map.weightageMap[coordinate[1] + i][coordinate[0]]:
                        self.__map.updateExploredMap(coordinate[1] + i, coordinate[0], 1)
                        self.__map.updateObstacleMap(coordinate[1] + i, coordinate[0], 0)
                        self.__map.updateWeightageMap(coordinate[1] + i, coordinate[0], weightage)
                weightage -= FRONT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + data - 1 <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1] + data - 1][coordinate[0]]:
                        self.__map.updateExploredMap(coordinate[1] + data - 1, coordinate[0], 1)
                        self.__map.updateObstacleMap(coordinate[1] + data - 1, coordinate[0], 1)
                        self.__map.updateWeightageMap(coordinate[1] + data - 1, coordinate[0], weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0]},{coordinate[1] + data - 1}')
                        # time.sleep(SLEEP_TIME)
        elif robot_bearing == Bearing.EAST:
            if data == -1:
                for i in range(MAX_FRONT):
                    if 0 <= coordinate[0] + i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] + i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] + i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] + i, weightage)
                    weightage -= FRONT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + i]:
                        self.__map.updateExploredMap(coordinate[1], coordinate[0] + i, 1)
                        self.__map.updateObstacleMap(coordinate[1], coordinate[0] + i, 0)
                        self.__map.updateWeightageMap(coordinate[1], coordinate[0] + i, weightage)
                weightage -= FRONT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] + data - 1 <= 14 and 0 <= coordinate[1] <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + data - 1]:
                        self.__map.updateExploredMap(coordinate[1], coordinate[0] + data - 1, 1)
                        self.__map.updateObstacleMap(coordinate[1], coordinate[0] + data - 1, 1)
                        self.__map.updateWeightageMap(coordinate[1], coordinate[0] + data - 1, weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0] + data - 1},{coordinate[1]}')
                        # time.sleep(SLEEP_TIME)
        elif robot_bearing == Bearing.SOUTH:
            if data == -1:
                for i in range(MAX_FRONT):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] - i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] - i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] - i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] - i, coordinate[0], weightage)
                    weightage -= FRONT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] - i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] - i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] - i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] - i, coordinate[0], weightage)
                    weightage -= FRONT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - (data - 1) <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1] - (data - 1)][coordinate[0]]:
                        self.__map.updateExploredMap(coordinate[1] - (data - 1), coordinate[0], 1)
                        self.__map.updateObstacleMap(coordinate[1] - (data - 1), coordinate[0], 1)
                        self.__map.updateWeightageMap(coordinate[1] - (data - 1), coordinate[0], weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0]},{coordinate[1] - (data - 1)}')
                        # time.sleep(SLEEP_TIME)
        else:
            if data == -1:
                for i in range(MAX_FRONT):
                    if 0 <= coordinate[0] - i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] - i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] - i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] - i, weightage)
                    weightage -= FRONT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] - i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] - i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] - i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] - i, weightage)
                    weightage -= FRONT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] - (data - 1) <= 14 and 0 <= coordinate[1] <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - (data - 1)]:
                        self.__map.updateExploredMap(coordinate[1], coordinate[0] - (data - 1), 1)
                        self.__map.updateObstacleMap(coordinate[1], coordinate[0] - (data - 1), 1)
                        self.__map.updateWeightageMap(coordinate[1], coordinate[0] - (data - 1), weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0] - (data - 1)},{coordinate[1]}')
                        # time.sleep(SLEEP_TIME)

    def update_left_map(self, data, coordinate, robot_bearing):
        weightage = 100
        if robot_bearing == Bearing.NORTH:
            if data == -1:
                for i in range(MAX_LEFT):
                    if 0 <= coordinate[0] - i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] - i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] - i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] - i, weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] - i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] - i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] - i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] - i, weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] - (data - 1) <= 14 and 0 <= coordinate[1] <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] - (data - 1)]:
                        self.__map.updateExploredMap(coordinate[1], coordinate[0] - (data - 1), 1)
                        self.__map.updateObstacleMap(coordinate[1], coordinate[0] - (data - 1), 1)
                        self.__map.updateWeightageMap(coordinate[1], coordinate[0] - (data - 1), weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0] - (data - 1)},{coordinate[1]}')
                        # time.sleep(SLEEP_TIME)
        elif robot_bearing == Bearing.EAST:
            if data == -1:
                for i in range(MAX_LEFT):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] + i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] + i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] + i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] + i, coordinate[0], weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] + i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] + i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] + i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] + i, coordinate[0], weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] + data - 1 <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1] + data - 1][coordinate[0]]:
                        self.__map.updateExploredMap(coordinate[1] + data - 1, coordinate[0], 1)
                        self.__map.updateObstacleMap(coordinate[1] + data - 1, coordinate[0], 1)
                        self.__map.updateWeightageMap(coordinate[1] + data - 1, coordinate[0], weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0]},{coordinate[1] + data - 1}')
                        # time.sleep(SLEEP_TIME)
        elif robot_bearing == Bearing.SOUTH:
            if data == -1:
                for i in range(MAX_LEFT):
                    if 0 <= coordinate[0] + i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] + i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] + i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] + i, weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] + i <= 14 and 0 <= coordinate[1] <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + i]:
                            self.__map.updateExploredMap(coordinate[1], coordinate[0] + i, 1)
                            self.__map.updateObstacleMap(coordinate[1], coordinate[0] + i, 0)
                            self.__map.updateWeightageMap(coordinate[1], coordinate[0] + i, weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] + data - 1 <= 14 and 0 <= coordinate[1] <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1]][coordinate[0] + data - 1]:
                        self.__map.updateExploredMap(coordinate[1], coordinate[0] + data - 1, 1)
                        self.__map.updateObstacleMap(coordinate[1], coordinate[0] + data - 1, 1)
                        self.__map.updateWeightageMap(coordinate[1], coordinate[0] + data - 1, weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0] + data - 1},{coordinate[1]}')
                        # time.sleep(SLEEP_TIME)
        else:
            if data == -1:
                for i in range(MAX_LEFT):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] - i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] - i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] - i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] - i, coordinate[0], weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
            else:
                for i in range(data - 1):
                    if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - i <= 19:
                        if weightage >= self.__map.weightageMap[coordinate[1] - i][coordinate[0]]:
                            self.__map.updateExploredMap(coordinate[1] - i, coordinate[0], 1)
                            self.__map.updateObstacleMap(coordinate[1] - i, coordinate[0], 0)
                            self.__map.updateWeightageMap(coordinate[1] - i, coordinate[0], weightage)
                    weightage -= LEFT_WEIGHTAGE_REDUCTION
                if 0 <= coordinate[0] <= 14 and 0 <= coordinate[1] - (data - 1) <= 19:
                    if weightage >= self.__map.weightageMap[coordinate[1] - (data - 1)][coordinate[0]]:
                        self.__map.updateExploredMap(coordinate[1] - (data - 1), coordinate[0], 1)
                        self.__map.updateObstacleMap(coordinate[1] - (data - 1), coordinate[0], 1)
                        self.__map.updateWeightageMap(coordinate[1] - (data - 1), coordinate[0], weightage)
                        # self.signalSendMsg.emit(f'OB|{coordinate[0]},{coordinate[1] - (data - 1)}')
                        # time.sleep(SLEEP_TIME)

    def update_expl_map(self, sensor_data, all_corners, robot_bearing):
        if robot_bearing == Bearing.NORTH:
            right_coordinate = all_corners[1]
            self.update_right_map(int(sensor_data[5]), right_coordinate, robot_bearing)

            front_coordinate = all_corners[0]
            self.update_front_map(int(sensor_data[2]), front_coordinate, robot_bearing)
            front_coordinate[0] += 1
            self.update_front_map(int(sensor_data[1]), front_coordinate, robot_bearing)
            front_coordinate[0] += 1
            self.update_front_map(int(sensor_data[0]), front_coordinate, robot_bearing)

            left_coordinate = all_corners[2]
            # bottom/middle left sensor is not placed correctly. this will create phantom block
            self.update_left_map(int(sensor_data[4]), left_coordinate, robot_bearing)
            left_coordinate[1] += 1
            left_coordinate[1] += 1
            self.update_left_map(int(sensor_data[3]), left_coordinate, robot_bearing)
        elif robot_bearing == Bearing.EAST:
            right_coordinate = all_corners[3]
            self.update_right_map(int(sensor_data[5]), right_coordinate, robot_bearing)

            front_coordinate = all_corners[1]
            self.update_front_map(int(sensor_data[2]), front_coordinate, robot_bearing)
            front_coordinate[1] -= 1
            self.update_front_map(int(sensor_data[1]), front_coordinate, robot_bearing)
            front_coordinate[1] -= 1
            self.update_front_map(int(sensor_data[0]), front_coordinate, robot_bearing)

            left_coordinate = all_corners[0]
            self.update_left_map(int(sensor_data[4]), left_coordinate, robot_bearing)
            left_coordinate[0] += 1
            left_coordinate[0] += 1
            self.update_left_map(int(sensor_data[3]), left_coordinate, robot_bearing)
        elif robot_bearing == Bearing.SOUTH:
            right_coordinate = all_corners[2]
            self.update_right_map(int(sensor_data[5]), right_coordinate, robot_bearing)

            front_coordinate = all_corners[3]
            self.update_front_map(int(sensor_data[2]), front_coordinate, robot_bearing)
            front_coordinate[0] -= 1
            self.update_front_map(int(sensor_data[1]), front_coordinate, robot_bearing)
            front_coordinate[0] -= 1
            self.update_front_map(int(sensor_data[0]), front_coordinate, robot_bearing)

            left_coordinate = all_corners[1]
            self.update_left_map(int(sensor_data[4]), left_coordinate, robot_bearing)
            left_coordinate[1] -= 1
            left_coordinate[1] -= 1
            self.update_left_map(int(sensor_data[3]), left_coordinate, robot_bearing)
        else:
            right_coordinate = all_corners[0]
            self.update_right_map(int(sensor_data[5]), right_coordinate, robot_bearing)

            front_coordinate = all_corners[2]
            self.update_front_map(int(sensor_data[2]), front_coordinate, robot_bearing)
            front_coordinate[1] += 1
            self.update_front_map(int(sensor_data[1]), front_coordinate, robot_bearing)
            front_coordinate[1] += 1
            self.update_front_map(int(sensor_data[0]), front_coordinate, robot_bearing)

            left_coordinate = all_corners[3]
            self.update_left_map(int(sensor_data[4]), left_coordinate, robot_bearing)
            left_coordinate[0] -= 1
            left_coordinate[0] -= 1
            self.update_left_map(int(sensor_data[3]), left_coordinate, robot_bearing)

    def setShortestPath(self, shortestPath):
        self.__shortestPath = shortestPath
        self.__spIndex = 0

    @pyqtSlot()
    def resetShortestPath(self):
        self.__shortestPath = None
        self.__spIndex = 0


class Forward(QObject):
    signalForward = pyqtSignal()
    finished = pyqtSignal()

    def __init__(self):
        super(Forward, self).__init__()
        self.__n = 0

    def set_n(self, n):
        self.__n = n

    @pyqtSlot()
    def run(self):
        for i in range(self.__n):
            print('[Arduino-Algo] Move robot forward')
            self.signalForward.emit()
            time.sleep(0.25)    # 0.25
        self.finished.emit()

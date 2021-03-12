from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtWidgets import QGraphicsEllipseItem
from constants import Bearing
from sensor import RobotSensorConfiguration


# Control the drawing of the robot
class Robot(QGraphicsEllipseItem):
    def __init__(self, x, y):
        super(Robot, self).__init__()
        self.__robotSize = 120
        self.__x = x
        self.__y = y
        self.__bearing = Bearing.NORTH

    @property
    def robotSize(self):
        return self.__robotSize

    @property
    def x(self):
        return self.__x

    @x.setter
    def x(self, xCoordinate):
        self.__x = xCoordinate

    @property
    def y(self):
        return self.__y

    @y.setter
    def y(self, yCoordinate):
        self.__y = yCoordinate

    @property
    def bearing(self):
        return self.__bearing

    @bearing.setter
    def bearing(self, b):
        self.__bearing = b

    def boundingRect(self):
        return QRectF(self.__x, self.__y, self.__robotSize, self.__robotSize)

    # Tells the QGraphicsEllipseItem how to paint the robot
    def paint(self, painter, option, widget):
        # Body
        painter.setBrush(Qt.red)
        painter.drawEllipse(self.__x, self.__y, self.__robotSize, self.__robotSize)

        # Redraw the robot if the robot rotates
        painter.setBrush(Qt.white)
        # Eye is facing top
        if self.__bearing == Bearing.NORTH:
            painter.drawEllipse(self.__x + 55, self.__y + 10, 10, 10)
        # Eye is facing right
        if self.__bearing == Bearing.EAST:
            painter.drawEllipse(self.__x + 100, self.__y + 55, 10, 10)
        # Eye is facing left
        if self.__bearing == Bearing.WEST:
            painter.drawEllipse(self.__x + 10, self.__y + 55, 10, 10)
        # Eye is facing bottom
        if self.__bearing == Bearing.SOUTH:
            painter.drawEllipse(self.__x + 55, self.__y + 100, 10, 10)


# A simulated robot
class SimRobot(Robot):
    def __init__(self, x, y, map, robotObject):
        super(SimRobot, self).__init__(x, y)
        self.__map = map
        self.__robotObject = robotObject
        self.__sensorConfig = RobotSensorConfiguration(self.__map)

    def resetPos(self):
        self.bearing = Bearing.NORTH
        self.update(self.boundingRect())
        self.moveRobot(0, -120)

    def moveRobot(self, x, y):
        if (-120 >= y >= -800) and (0 <= x <= 480):
            self.x = x
            self.y = y
            self.setRect(self.x, self.y, self.robotSize, self.robotSize)

    def moveRobotForward(self):
        if self.bearing == Bearing.NORTH:
            # Robot move up
            self.moveRobot(self.x, self.y - 40)
        elif self.bearing == Bearing.EAST:
            # Robot move right
            self.moveRobot(self.x + 40, self.y)
        elif self.bearing == Bearing.SOUTH:
            # Robot move down
            self.moveRobot(self.x, self.y + 40)
        elif self.bearing == Bearing.WEST:
            # Robot move left
            self.moveRobot(self.x - 40, self.y)

    def moveRobotBackward(self):
        if self.bearing == Bearing.NORTH:
            # Robot reverse down
            self.moveRobot(self.x, self.y + 40)
        elif self.bearing == Bearing.EAST:
            # Robot reverse left
            self.moveRobot(self.x - 40, self.y)
        elif self.bearing == Bearing.SOUTH:
            # Robot reverse up
            self.moveRobot(self.x, self.y - 40)
        elif self.bearing == Bearing.WEST:
            # Robot reverse right
            self.moveRobot(self.x + 40, self.y)

    def rotateRobotRight(self):
        self.bearing = Bearing.rotateRight(self.bearing)
        self.update(self.boundingRect())

    def rotateRobotLeft(self):
        self.bearing = Bearing.rotateLeft(self.bearing)
        self.update(self.boundingRect())

    def sense(self):
        self.__sensorConfig.senseAll(self.bearing, self.x, self.y)
        self.checkFrontLeft()

    def get_all_corners(self):
        x = int(self.x / 40)
        y = int(abs(self.y) / 40)

        # the x and y is in col, row coordinate system
        topLeftCorner = [x, y]
        topRightCorner = [x + 3, y - 1]
        bottomLeftCorner = [x - 1, y - 3]
        bottomRightCorner = [x + 2, y - 4]
        allCorners = [topLeftCorner, topRightCorner, bottomLeftCorner, bottomRightCorner]

        return allCorners

    # emits a signal with a dictionary
    # { 'F' : <0,1>, 'L' : <0,1> } 0 -> No obstacle, 1 -> obstacle
    def checkFrontLeft(self):
        allCorners = self.get_all_corners()

        frontLeftDict = {'F': 0, 'L': 0}
        topLeftCorner = allCorners[0]
        topRightCorner = allCorners[1]
        bottomLeftCorner = allCorners[2]
        bottomRightCorner = allCorners[3]

        if self.bearing == Bearing.NORTH:
            # Check the front
            if topLeftCorner[1] > 19:  # out of arena range
                frontLeftDict['F'] = 1
            else:
                for col in range(topLeftCorner[0], topLeftCorner[0] + 3):
                    if self.__map.obstacleMap[topLeftCorner[1]][col] == 1:
                        frontLeftDict['F'] = 1
                        break
            # Check the left
            if bottomLeftCorner[0] < 0:  # out of arena range
                frontLeftDict['L'] = 1
            else:
                for row in range(bottomLeftCorner[1], bottomLeftCorner[1] + 3):
                    if self.__map.obstacleMap[row][bottomLeftCorner[0]] == 1:
                        frontLeftDict['L'] = 1
                        break
        elif self.bearing == Bearing.EAST:
            # Check the front
            if topRightCorner[0] > 14:  # out of arena range
                frontLeftDict['F'] = 1
            else:
                for row in range(topRightCorner[1], topRightCorner[1] - 3, -1):
                    if self.__map.obstacleMap[row][topRightCorner[0]] == 1:
                        frontLeftDict['F'] = 1
                        break
            # Check the left
            if topLeftCorner[1] > 19:  # out of arena range
                frontLeftDict['L'] = 1
            else:
                for col in range(topLeftCorner[0], topLeftCorner[0] + 3):
                    if self.__map.obstacleMap[topLeftCorner[1]][col] == 1:
                        frontLeftDict['L'] = 1
                        break
        elif self.bearing == Bearing.SOUTH:
            # Check the front
            if bottomRightCorner[1] < 0:  # out of arena range
                frontLeftDict['F'] = 1
            else:
                for col in range(bottomRightCorner[0], bottomRightCorner[0] - 3, -1):
                    if self.__map.obstacleMap[bottomRightCorner[1]][col] == 1:
                        frontLeftDict['F'] = 1
                        break
            # Check the left
            if topRightCorner[0] > 14:  # out of arena range
                frontLeftDict['L'] = 1
            else:
                for row in range(topRightCorner[1], topRightCorner[1] - 3, -1):
                    if self.__map.obstacleMap[row][topRightCorner[0]] == 1:
                        frontLeftDict['L'] = 1
                        break
        elif self.bearing == Bearing.WEST:
            # Check the front
            if bottomLeftCorner[0] < 0:  # out of arena range
                frontLeftDict['F'] = 1
            else:
                for row in range(bottomLeftCorner[1], bottomLeftCorner[1] + 3):
                    if self.__map.obstacleMap[row][bottomLeftCorner[0]] == 1:
                        frontLeftDict['F'] = 1
                        break
            # Check the left
            if bottomRightCorner[1] < 0:  # out of arena range
                frontLeftDict['L'] = 1
            else:
                for col in range(bottomRightCorner[0], bottomRightCorner[0] - 3, -1):
                    if self.__map.obstacleMap[bottomRightCorner[1]][col] == 1:
                        frontLeftDict['L'] = 1
                        break
        self.__robotObject.emitFrontLeft(frontLeftDict, allCorners, self.__map.exploredMap, self.__map.obstacleMap, self.bearing)

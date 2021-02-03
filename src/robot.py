from PyQt5.QtCore import QRectF, Qt, QMutex, QMutexLocker
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
    def __init__(self, x, y, map):
        super(SimRobot, self).__init__(x, y)
        self.__mutex = QMutex()
        self.__map = map
        self.__sensorConfig = RobotSensorConfiguration(self.__map)

    def resetPos(self):
        self.bearing = Bearing.NORTH
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

    def noOfLeftMove(self):
        with QMutexLocker(self.__mutex):
            x = int(self.x / 40)
            y = int(abs(self.y) / 40)

            topLeftCorner = [x, y]
            topRightCorner = [x + 3, y - 1]
            bottomLeftCorner = [x - 1, y - 3]
            bottomRightCorner = [x + 2, y - 4]

            emptyLeft = 0

            if self.bearing == Bearing.NORTH:
                obstacle = False
                for col in range(bottomLeftCorner[0], bottomLeftCorner[0] - 2, -1):
                    if col < 0:   # out of arena range
                        break
                    else:   # check if there is obstacle on the left
                        for row in range(bottomLeftCorner[1], bottomLeftCorner[1] + 3):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyLeft = emptyLeft + 1
            elif self.bearing == Bearing.EAST:
                obstacle = False
                for row in range(topLeftCorner[1], topLeftCorner[1] + 2):
                    if row > 19:    # out of arena range
                        break
                    else:   # check if there is obstacle on the left
                        for col in range(topLeftCorner[0], topLeftCorner[0] + 3):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyLeft = emptyLeft + 1
            elif self.bearing == Bearing.SOUTH:
                obstacle = False
                for col in range(topRightCorner[0], topRightCorner[0] + 2):
                    if col > 14:    # out of arena range
                        break
                    else:   # check if there is obstacle on the left
                        for row in range(topRightCorner[1], topRightCorner[1] - 3, -1):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyLeft = emptyLeft + 1
            elif self.bearing == Bearing.WEST:
                obstacle = False
                for row in range(bottomRightCorner[1], bottomRightCorner[1] - 2, -1):
                    if row < 0:     # out of arena range
                        break
                    else:   # check if there is obstacle on the left
                        for col in range(bottomRightCorner[0], bottomRightCorner[0] - 3, -1):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyLeft = emptyLeft + 1
            return emptyLeft

    def noOfForwardMove(self):
        with QMutexLocker(self.__mutex):
            x = int(self.x / 40)
            y = int(abs(self.y) / 40)

            topLeftCorner = [x, y]
            topRightCorner = [x + 3, y - 1]
            bottomLeftCorner = [x - 1, y - 3]
            bottomRightCorner = [x + 2, y - 4]

            emptyForward = 0

            if self.bearing == Bearing.NORTH:
                obstacle = False
                for row in range(topLeftCorner[1], topLeftCorner[1] + 2):
                    if row > 19:
                        break
                    else:
                        for col in range(topLeftCorner[0], topLeftCorner[0] + 3):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyForward = emptyForward + 1
            elif self.bearing == Bearing.EAST:
                obstacle = False
                for col in range(topRightCorner[0], topRightCorner[0] + 2):
                    if col > 14:
                        break
                    else:
                        for row in range(topRightCorner[1], topRightCorner[1] - 3, -1):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyForward = emptyForward + 1
            elif self.bearing == Bearing.SOUTH:
                obstacle = False
                for row in range(bottomRightCorner[1], bottomRightCorner[1] - 2, -1):
                    if row < 0:
                        break
                    else:
                        for col in range(bottomRightCorner[0], bottomRightCorner[0] - 3, -1):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyForward = emptyForward + 1
            elif self.bearing == Bearing.WEST:
                obstacle = False
                for col in range(bottomLeftCorner[0], bottomLeftCorner[0] - 2, -1):
                    if col < 0:
                        break
                    else:
                        for row in range(bottomLeftCorner[1], bottomLeftCorner[1] + 3):
                            if not obstacle:
                                if self.__map.obstacleMap[row][col] == 1:
                                    obstacle = True
                                    break
                            else:
                                break
                        if not obstacle:
                            emptyForward = emptyForward + 1
            return emptyForward
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
    def __init__(self, x, y, map):
        super(SimRobot, self).__init__(x, y)
        self.__sensorConfig = RobotSensorConfiguration(map)

    def moveRobot(self, x, y):
        if (-120 >= y >= -800) and (0 <= x <= 480):
            self.x = x
            self.y = y
            super(SimRobot, self).setRect(self.x, self.y, self.robotSize, self.robotSize)

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

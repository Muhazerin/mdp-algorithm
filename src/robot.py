from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtWidgets import QGraphicsEllipseItem
from constants import Bearing


# Control the drawing of the robot
class Robot(QGraphicsEllipseItem):
    def __init__(self, x, y):
        super(Robot, self).__init__()
        self.__x = x
        self.__y = y
        self.__bearing = Bearing.NORTH

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def bearing(self):
        return self.__bearing

    @bearing.setter
    def bearing(self, b):
        self.__bearing = b

    # To be overridden
    def setPos(self, x, y):
        pass

    def boundingRect(self):
        return QRectF(self.__x, self.__y, 120, 120)

    # Tells the QGraphicsEllipseItem how to pain the robot
    def paint(self, painter, option, widget):
        # Body
        painter.setBrush(Qt.red)
        painter.drawEllipse(self.__x, self.__y, 120, 120)

        # Redraw the robot if the robot rotates
        painter.setBrush(Qt.white)
        # Eye is facing top
        if (self.__bearing == Bearing.NORTH):
            painter.drawEllipse(self.__x + 55, self.__y + 10, 10, 10)
        # Eye is facing right
        if (self.__bearing == Bearing.EAST):
            painter.drawEllipse(self.__x + 100, self.__y + 55, 10, 10)
        # Eye is facing left
        if (self.__bearing == Bearing.WEST):
            painter.drawEllipse(self.__x + 10, self.__y + 55, 10, 10)
        # Eye is facing bottom
        if (self.__bearing == Bearing.SOUTH):
            painter.drawEllipse(self.__x + 55, self.__y + 100, 10, 10)



class SimRobot(Robot):
    def __init__(self, x, y):
        super(SimRobot, self).__init__(x, y)

    def setPos(self, x, y):
        if (y <= -120 and y >= -800) and (x >= 0 and x <= 480):
            self.__x = x
            self.__y = y
            self.setRect(self.__x, self.__y, 120, 120)

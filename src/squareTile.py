from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPen, QBrush
from PyQt5.QtWidgets import QGraphicsRectItem
from constants import TileType


# Defines the parameter of the square tile in the map
class SquareTile(QGraphicsRectItem):
    def __init__(self, x, y, tileType):
        super(SquareTile, self).__init__()
        self.__tileSize = 40
        self.__x = x
        self.__y = y
        self.__tileType = tileType

        self.setPen(QPen(Qt.black))
        self.updateTileType()
        self.setRect(self.__x, self.__y, self.__tileSize, self.__tileSize)

    @property
    def tileType(self):
        return self.__tileType

    @tileType.setter
    def tileType(self, tt):
        self.__tileType = tt
        self.updateTileType()

    def updateTileType(self):
        if self.__tileType == TileType.UNEXPLORED:
            self.setBrush(QBrush(Qt.lightGray))
        elif self.__tileType == TileType.EXPLORED:
            self.setBrush(QBrush(Qt.white))
        elif self.__tileType == TileType.OBSTACLE:
            self.setBrush(QBrush(Qt.black))
        elif self.__tileType == TileType.START:
            self.setBrush(QBrush(Qt.blue))
        elif self.__tileType == TileType.WAYPOINT:
            self.setBrush(QBrush(Qt.yellow))
        else:
            self.setBrush(QBrush(Qt.green))

    def boundingRect(self):
        return QRectF(self.__x, self.__y, self.__tileSize, self.__tileSize)

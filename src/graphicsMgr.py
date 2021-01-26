from map import Map
from squareTile import SquareTile
from PyQt5.QtCore import Qt
from robot import SimRobot
from constants import Bearing, TileType


# Handles the graphics/drawing that user see on the app
class GraphicsMgr:
    def __init__(self, scene):
        self.__scene = scene
        self.__map = Map()
        self.__map.addObserver(self)

        self.initMap()

    # Initialize the square tiles on the map and the robot
    def initMap(self):
        # set the 15x20 map
        # Qt is weird...
        # left to right is -x to +x
        # down to up is +y to -y
        for y in range(-40, -801, -40):
            for x in range(0, 561, 40):
                square_tile = SquareTile(x, y, TileType.UNEXPLORED)
                self.__scene.addItem(square_tile)

        # Retrieve the tiles from the scene in descending order (0 = descending order)
        self.__tileList = self.__scene.items(0)
        # Color the START blue
        self.changeTile(0, 3, TileType.START)
        self.changeTile(15, 18, TileType.START)
        self.changeTile(30, 33, TileType.START)
        # Color the GOAl green
        self.changeTile(267, 270, TileType.GOAL)
        self.changeTile(282, 285, TileType.GOAL)
        self.changeTile(297, 300, TileType.GOAL)

        # Initialize the robot
        self.__robot = SimRobot(0, -120)
        self.__scene.addItem(self.__robot)

    def resetMap(self):
        self.__map.reset = True

    def changeTile(self, start, end, tt):
        for i in range(start, end):
            self.__tileList[i].tileType = tt

    def rotateRobotRight(self):
        self.__robot.bearing = Bearing.rotateRight(self.__robot.bearing)
        self.__robot.update(self.__robot.boundingRect())

    def rotateRobotLeft(self):
        self.__robot.bearing = Bearing.rotateLeft(self.__robot.bearing)
        self.__robot.update(self.__robot.boundingRect())

    def moveRobotForward(self):
        if self.__robot.bearing == Bearing.NORTH:
            # Robot moves up
            self.__robot.setPos(self.__robot.x, self.__robot.y - 40)
        elif self.__robot.bearing == Bearing.EAST:
            # Robot moves right
            print(f"x: {self.__robot.x + 40}, y: {self.__robot.y}")
            self.__robot.setPos(self.__robot.x + 40, self.__robot.y)
        elif self.__robot.bearing == Bearing.SOUTH:
            # Robot moves down
            self.__robot.setPos(self.__robot.x, self.__robot.y + 40)
        elif self.__robot.bearing == Bearing.WEST:
            # Robot moves left
            self.__robot.setPos(self.__robot.x - 40, self.__robot.y)

    def moveRobotBackward(self):
        if (self.__robot.bearing == Bearing.NORTH):
            # Robot moves up
            self.__robot.setPos(self.__robot.x, self.__robot.y + 40)
        elif (self.__robot.bearing == Bearing.EAST):
            # Robot moves right
            self.__robot.setPos(self.__robot.x - 40, self.__robot.y)
        elif (self.__robot.bearing == Bearing.SOUTH):
            # Robot moves down
            self.__robot.setPos(self.__robot.x, self.__robot.y - 40)
        elif (self.__robot.bearing == Bearing.WEST):
            # Robot moves left
            self.__robot.setPos(self.__robot.x + 40, self.__robot.y)

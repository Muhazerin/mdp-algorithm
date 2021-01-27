from squareTile import SquareTile
from constants import TileType, MapConstant


# Handles the graphics/drawing that user see on the app
class GraphicsMgr:
    def __init__(self, scene, robot, map):
        self.__scene = scene
        self.__robot = robot
        self.__dontTouchMapList = MapConstant.getMapStartList() + MapConstant.getMapGoalList()
        map.addObserver(self)

        self.initMap()

        # Add the robot to the scene
        self.__scene.addItem(self.__robot)
        self.__robot.sense()

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
        self.changeTile(MapConstant.getMapStartList(), TileType.START)
        # Color the GOAl green
        self.changeTile(MapConstant.getMapGoalList(), TileType.GOAL)

    def changeTile(self, mapList, tt):
        for i in mapList:
            self.__tileList[i].tileType = tt

    def updateMap(self, index, tt):
        if index not in self.__dontTouchMapList:
            self.__tileList[index].tileType = tt


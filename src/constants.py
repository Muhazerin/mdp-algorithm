from enum import IntEnum


MIN_Y = 0
MAX_Y = 19
MIN_X = 0
MAX_X = 14

class MapConstant(IntEnum):
    # Returns a list that contains that index of START
    @staticmethod
    def getMapStartList():
        return [0, 1, 2, 15, 16, 17, 30, 31, 32]

    # Returns a list that contains that index of GOAL
    @staticmethod
    def getMapGoalList():
        return [267, 268, 269, 282, 283, 284, 297, 298, 299]


class Bearing(IntEnum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6

    @staticmethod
    def rotateRight(currentBearing):
        return Bearing((currentBearing.value + 2) % 8)

    @staticmethod
    def rotateLeft(currentBearing):
        return Bearing((currentBearing.value + 6) % 8)


class TileType(IntEnum):
    UNEXPLORED = 1
    EXPLORED = 2
    OBSTACLE = 3
    START = 4
    GOAL = 5


class SensorLocation(IntEnum):
    FRONT = 1
    RIGHT = 2
    BACK = 3
    LEFT = 4

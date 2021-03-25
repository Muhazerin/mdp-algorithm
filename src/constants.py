from enum import IntEnum


MIN_Y = 0
MAX_Y = 19
MIN_X = 0
MAX_X = 14


class AlgoStatus(IntEnum):
    SEEK_GOAL = 0
    SEEK_HOME = 1
    FP_UNEXPLORED_SEARCH = 2
    FP_UNEXPLORED_SEEK = 3
    LEFT_WALL_HUG = 4
    FP_HOME_SEARCH = 5
    FP_HOME_SEEK = 6


class ImgRecogAlgoStatus(IntEnum):
    SEEK_GOAL = 0
    SEEK_HOME = 1
    FP_TO_OBSTACLE = 2
    LEFT_WALL_HUG = 3
    SEARCH_OBSTACLE = 4


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
    UNEXPLORED = 1              # light gray
    EXPLORED = 2                # white
    UNEXPLORED_OBSTACLE = 3     # dark grey
    EXPLORED_OBSTACLE = 4       # black
    WAYPOINT = 5                # yellow
    START = 6                   # blue
    UNEXPLORED_GOAL = 7         # dark green
    EXPLORED_GOAL = 8           # green


class SensorLocation(IntEnum):
    FRONT = 1
    RIGHT = 2
    BACK = 3
    LEFT = 4

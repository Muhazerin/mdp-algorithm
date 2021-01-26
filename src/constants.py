from enum import IntEnum


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
import constants
from constants import SensorLocation, Bearing


class RobotSensorConfiguration:
    def __init__(self, map):
        self.__sensorList = [
            SimIrSensor(SensorLocation.FRONT, map),
            SimIrSensor(SensorLocation.LEFT, map)
        ]

    def senseAll(self, robotBearing, robotX, robotY):
        for sensor in self.__sensorList:
            sensor.sense(robotBearing, robotX, robotY)


class SimIrSensor:
    def __init__(self, sensorLocation, map):
        self.__sensorLocation = sensorLocation
        self.__map = map

    @property
    def sensorLocation(self):
        return self.__sensorLocation

    def updateMap(self, j, i):
        obstacle = False
        if (constants.MIN_X <= i <= constants.MAX_X) and (constants.MIN_Y <= j <= constants.MAX_Y):
            self.__map.updateExploredMap(j, i, 1)
            if self.__map.simExplObstacleMap[j][i] == 0:
                self.__map.updateObstacleMap(j, i, 0)
            else:
                obstacle = True
                self.__map.updateObstacleMap(j, i, 1)
        return obstacle

    def senseNorth(self, x, y):
        for i in range(x, x + 3):
            for j in range(y, y + 2):
                obstacle = self.updateMap(j, i)
                if obstacle:
                    break

    def senseEast(self, x, y):
        x = x + 3
        y = y - 1
        for j in range(y, y - 3, -1):
            for i in range(x, x + 2):
                obstacle = self.updateMap(j, i)
                if obstacle:
                    break

    def senseWest(self, x, y):
        x = x - 1
        y = y - 1
        for j in range(y, y - 3, -1):
            for i in range(x, x - 2, -1):
                obstacle = self.updateMap(j, i)
                if obstacle:
                    break

    def senseSouth(self, x, y):
        y = y - 4
        for i in range(x, x + 3):
            for j in range(y, y - 2, -1):
                obstacle = self.updateMap(j, i)
                if obstacle:
                    break

    def sense(self, robotBearing, robotX, robotY):
        x = int(robotX / 40)
        y = int(abs(robotY) / 40)

        if robotBearing == Bearing.NORTH:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseNorth(x, y)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseEast(x, y)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseSouth(x, y)
            else:
                self.senseWest(x, y)
        elif robotBearing == Bearing.EAST:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseEast(x, y)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseSouth(x, y)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseWest(x, y)
            else:
                self.senseNorth(x, y)
        elif robotBearing == Bearing.SOUTH:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseSouth(x, y)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseWest(x, y)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseNorth(x, y)
            else:
                self.senseEast(x, y)
        else:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseWest(x, y)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseNorth(x, y)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseEast(x, y)
            else:
                self.senseSouth(x, y)

import constants
from constants import SensorLocation, Bearing


class RobotSensorConfiguration:
    def __init__(self, map):
        self.__sensorList = [
            SimShortRangeSensor(SensorLocation.FRONT, 0, map),
            SimShortRangeSensor(SensorLocation.FRONT, 1, map),
            SimShortRangeSensor(SensorLocation.FRONT, 2, map),
            SimShortRangeSensor(SensorLocation.LEFT, 1, map),
            SimShortRangeSensor(SensorLocation.LEFT, 2, map),
            SimLongRangeSensor(SensorLocation.RIGHT, 0, map)
        ]

    def senseAll(self, robotBearing, robotX, robotY):
        x = int(robotX / 40)
        y = int(abs(robotY) / 40)
        topLeftCorner = [x, y]
        topRightCorner = [x + 3, y - 1]
        bottomLeftCorner = [x - 1, y - 3]
        bottomRightCorner = [x + 2, y - 4]

        for sensor in self.__sensorList:
            sensor.sense(robotBearing, topLeftCorner, topRightCorner, bottomLeftCorner, bottomRightCorner)


class IrSensor:
    def __init__(self, sensorLocation, offset, map):
        self.__sensorLocation = sensorLocation
        # The offset is dynamic. If can be x offset or y offset depending on the robot bearing
        self.__offset = offset
        self.__map = map

    def updateMap(self, row, col):
        obstacle = False
        if (constants.MIN_X <= col <= constants.MAX_X) and (constants.MIN_Y <= row <= constants.MAX_Y):
            self.__map.updateExploredMap(row, col, 1)
            if self.__map.simExplObstacleMap[row][col] == 0:
                self.__map.updateObstacleMap(row, col, 0)
            else:
                obstacle = True
                self.__map.updateObstacleMap(row, col, 1)
        return obstacle

    def senseFront(self, col, row, sensorRange):
        for jRow in range(row, row + sensorRange):
            obstacle = self.updateMap(jRow, col)
            if obstacle:
                break

    def senseRight(self, col, row, sensorRange):
        for iCol in range(col, col + sensorRange):
            obstacle = self.updateMap(row, iCol)
            if obstacle:
                break

    def senseBack(self, col, row, sensorRange):
        for jRow in range(row, row - sensorRange, -1):
            obstacle = self.updateMap(jRow, col)
            if obstacle:
                break

    def senseLeft(self, col, row, sensorRange):
        for iCol in range(col, col - sensorRange, -1):
            obstacle = self.updateMap(row, iCol)
            if obstacle:
                break

    def sense(self, robotBearing, topLeftCorner, topRightCorner, bottomLeftCorner, bottomRightCorner, sensorRange):
        # Reposition x and y to show the robot's front left corner
        if robotBearing == Bearing.NORTH:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseFront(topLeftCorner[0] + self.__offset, topLeftCorner[1], sensorRange)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseRight(topRightCorner[0], topRightCorner[1] - self.__offset, sensorRange)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseBack(bottomRightCorner[0] - self.__offset, bottomRightCorner[1], sensorRange)
            elif self.__sensorLocation == SensorLocation.LEFT:
                self.senseLeft(bottomLeftCorner[0], bottomLeftCorner[1] + self.__offset, sensorRange)
        elif robotBearing == Bearing.EAST:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseRight(topRightCorner[0], topRightCorner[1] - self.__offset, sensorRange)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseBack(bottomRightCorner[0] - self.__offset, bottomRightCorner[1], sensorRange)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseLeft(bottomLeftCorner[0], bottomLeftCorner[1] + self.__offset, sensorRange)
            elif self.__sensorLocation == SensorLocation.LEFT:
                self.senseFront(topLeftCorner[0] + self.__offset, topLeftCorner[1], sensorRange)
        elif robotBearing == Bearing.SOUTH:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseBack(bottomRightCorner[0] - self.__offset, bottomRightCorner[1], sensorRange)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseLeft(bottomLeftCorner[0], bottomLeftCorner[1] + self.__offset, sensorRange)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseFront(topLeftCorner[0] + self.__offset, topLeftCorner[1], sensorRange)
            elif self.__sensorLocation == SensorLocation.LEFT:
                self.senseRight(topRightCorner[0], topRightCorner[1] - self.__offset, sensorRange)
        elif robotBearing == Bearing.WEST:
            if self.__sensorLocation == SensorLocation.FRONT:
                self.senseLeft(bottomLeftCorner[0], bottomLeftCorner[1] + self.__offset, sensorRange)
            elif self.__sensorLocation == SensorLocation.RIGHT:
                self.senseFront(topLeftCorner[0] + self.__offset, topLeftCorner[1], sensorRange)
            elif self.__sensorLocation == SensorLocation.BACK:
                self.senseRight(topRightCorner[0], topRightCorner[1] - self.__offset, sensorRange)
            elif self.__sensorLocation == SensorLocation.LEFT:
                self.senseBack(bottomRightCorner[0] - self.__offset, bottomRightCorner[1], sensorRange)


class SimShortRangeSensor(IrSensor):
    def __init__(self, sensorLocation, offset, map):
        super(SimShortRangeSensor, self).__init__(sensorLocation, offset, map)
        self.__sensorRange = 2

    def sense(self, robotBearing, topLeftCorner, topRightCorner, bottomLeftCorner, bottomRightCorner):
        super(SimShortRangeSensor, self).sense(robotBearing, topLeftCorner, topRightCorner, bottomLeftCorner,
                                               bottomRightCorner, self.__sensorRange)


class SimLongRangeSensor(IrSensor):
    def __init__(self, sensorLocation, offset, map):
        super(SimLongRangeSensor, self).__init__(sensorLocation, offset, map)
        self.__sensorRange = 4

    def sense(self, robotBearing, topLeftCorner, topRightCorner, bottomLeftCorner, bottomRightCorner):
        super(SimLongRangeSensor, self).sense(robotBearing, topLeftCorner, topRightCorner, bottomLeftCorner,
                                              bottomRightCorner, self.__sensorRange)
